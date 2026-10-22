#include "section.h"

#include "shape_grammar.h"   // offsetPlanPublic
#include <clipper2/clipper.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

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

void ring(const std::vector<Edge>& edges, Real tol, Poly2& pts, std::vector<int>* edgeOf, Real bay = 0) {
    for (std::size_t i = 0; i < edges.size(); ++i) {
        const Edge& e = edges[i];
        pts.push_back(e.a);
        if (edgeOf) edgeOf->push_back(static_cast<int>(i));
        if (e.kind != Edge::Kind::Arc) continue;
        const ArcGeom g = arcOf(e);
        if (!g.ok) continue;
        // chords rising at most `tol` off the arc: the half angle each may span; for a building, at least as many
        // as the facade's bay asks (one even bay per chord)
        const Real halfMax = std::acos(std::max(Real(-1), std::min(Real(1), 1.0 - std::max(tol, Real(1e-3)) / g.r)));
        int n = std::max(1, static_cast<int>(std::ceil(std::fabs(g.sweep) / (2.0 * halfMax))));
        if (bay > 0.5) n = std::max(n, static_cast<int>(std::lround(std::fabs(g.sweep) * g.r / bay)));
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

Poly2 Section::tessellateForBays(Real bay, Real tol) const {
    Poly2 pts;
    ring(outer, tol, pts, nullptr, bay);
    return pts;
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

Section Section::chamfer(std::size_t corner, Real c) const {
    const std::size_t n = outer.size();
    if (n < 3 || corner >= n || c <= 0) return *this;
    const Edge& in = outer[(corner + n - 1) % n];
    const Edge& out = outer[corner];
    if (in.kind != Edge::Kind::Line || out.kind != Edge::Kind::Line) return *this;
    const Real cIn = std::min(c, (in.b - in.a).length() * 0.5), cOut = std::min(c, (out.b - out.a).length() * 0.5);
    Section s = *this;
    Edge& ein = s.outer[(corner + n - 1) % n];
    Edge& eout = s.outer[corner];
    ein.b = ein.b - normalize(ein.b - ein.a) * cIn;
    eout.a = eout.a + normalize(eout.b - eout.a) * cOut;
    Edge cut;
    cut.a = ein.b;
    cut.b = eout.a;
    s.outer.insert(s.outer.begin() + static_cast<long>(corner), cut);
    return s;
}

Section Section::notch(std::size_t edge, Real a0, Real a1, Real depth) const {
    const std::size_t n = outer.size();
    if (n < 3 || edge >= n || outer[edge].kind != Edge::Kind::Line) return *this;
    const Edge& e = outer[edge];
    const Vec2 d = normalize(e.b - e.a);
    const Real L = (e.b - e.a).length();
    a0 = std::max(Real(0), std::min(a0, L));
    a1 = std::max(a0, std::min(a1, L));
    if (a1 - a0 < 1e-6 || depth <= 0) return *this;
    const Vec2 nIn(-d.y, d.x);   // inward of a CCW edge
    const Vec2 p0 = e.a + d * a0, p1 = e.a + d * a1;
    const Vec2 q0 = p0 + nIn * depth, q1 = p1 + nIn * depth;
    std::vector<Edge> es;
    for (std::size_t i = 0; i < n; ++i) {
        if (i != edge) { es.push_back(outer[i]); continue; }
        auto line = [](const Vec2& a, const Vec2& b) { Edge l; l.a = a; l.b = b; return l; };
        if (a0 > 1e-6) es.push_back(line(e.a, p0));
        es.push_back(line(p0, q0));
        es.push_back(line(q0, q1));
        es.push_back(line(q1, p1));
        if (L - a1 > 1e-6) es.push_back(line(p1, e.b));
    }
    Section s;
    s.outer = es;
    s.holes = holes;
    return s;
}

namespace {

constexpr double kClipScale = 1000.0;   // a millimetre grid, as the lanes' geometry

Clipper2Lib::Path64 toPath(const Poly2& r) {
    Clipper2Lib::Path64 p;
    p.reserve(r.size());
    for (const Vec2& v : r) p.emplace_back(static_cast<int64_t>(std::llround(v.x * kClipScale)), static_cast<int64_t>(std::llround(v.y * kClipScale)));
    return p;
}

Poly2 fromPath(const Clipper2Lib::Path64& p) {
    Poly2 r;
    r.reserve(p.size());
    for (const auto& q : p) r.emplace_back(static_cast<Real>(q.x) / kClipScale, static_cast<Real>(q.y) / kClipScale);
    return r;
}

Section boolean(const Section& a, const Section& b, Clipper2Lib::ClipType op, Real tol) {
    Clipper2Lib::Paths64 subj, clip;
    subj.push_back(toPath(a.tessellate(tol)));
    for (const Poly2& h : a.holesTessellated(tol)) subj.push_back(toPath(h));
    clip.push_back(toPath(b.tessellate(tol)));
    for (const Poly2& h : b.holesTessellated(tol)) clip.push_back(toPath(h));
    Clipper2Lib::PolyTree64 tree;
    Clipper2Lib::Clipper64 c;
    c.PreserveCollinear(false);
    c.AddSubject(subj);
    c.AddClip(clip);
    c.Execute(op, Clipper2Lib::FillRule::NonZero, tree);
    // the largest outline, with its holes
    const Clipper2Lib::PolyPath64* best = nullptr;
    double bestArea = 0;
    for (const auto& child : tree) {
        const double ar = std::fabs(Clipper2Lib::Area(child->Polygon()));
        if (ar > bestArea) { bestArea = ar; best = child.get(); }
    }
    if (!best) return Section{};
    Poly2 outline = fromPath(best->Polygon());
    ensureCCW(outline);
    Section s = Section::retagArcs(outline);
    for (const auto& hole : *best) {
        Poly2 h = fromPath(hole->Polygon());
        Section hs = Section::retagArcs(h);
        s.holes.push_back(hs.outer);
    }
    return s;
}

}  // namespace

Section Section::unite(const Section& other, Real tol) const { return boolean(*this, other, Clipper2Lib::ClipType::Union, tol); }
Section Section::subtract(const Section& other, Real tol) const { return boolean(*this, other, Clipper2Lib::ClipType::Difference, tol); }
Section Section::intersect(const Section& other, Real tol) const { return boolean(*this, other, Clipper2Lib::ClipType::Intersection, tol); }

Section Section::retagArcs(const Poly2& outline) {
    const std::size_t n = outline.size();
    Section s;
    if (n < 3) return s;
    std::vector<Real> len(n), turn(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = outline[i], b = outline[(i + 1) % n], c = outline[(i + 2) % n];
        len[i] = (b - a).length();
        const Vec2 d0 = normalize(b - a), d1 = normalize(c - b);
        turn[i] = std::atan2(cross(d0, d1), dot(d0, d1));   // from edge i into edge i + 1
    }
    auto continues = [&](std::size_t i, std::size_t j) {   // edge j carries on edge i's arc
        const Real t0 = turn[i], t1 = turn[(j + n - 1) % n];
        return len[i] > 1e-6 && std::fabs(len[j] - len[i]) < 0.02 * len[i] && std::fabs(t0) > 0.0175 && std::fabs(t0) < 0.80 &&
               std::fabs(t1 - t0) < 0.0175;
    };
    std::vector<int> group(n, -1);
    std::vector<std::pair<std::size_t, std::size_t>> runs;   // first edge, count
    for (std::size_t i = 0; i < n; ++i) {
        if (group[i] >= 0) continue;
        if (continues((i + n - 1) % n, i) && group[(i + n - 1) % n] < 0) continue;   // not the run's start
        std::size_t count = 1;
        Real sweep = 0;
        while (count < n) {
            const std::size_t j = (i + count) % n;
            if (group[j] >= 0 || !continues((i + count - 1) % n, j)) break;
            if (std::fabs(sweep + turn[(i + count - 1) % n]) > kPi + 0.02) break;
            sweep += turn[(i + count - 1) % n];
            ++count;
        }
        if (count < 2) continue;
        for (std::size_t k = 0; k < count; ++k) group[(i + k) % n] = static_cast<int>(runs.size());
        runs.push_back({i, count});
    }
    // the edges in order: a run becomes one arc through its ends, the chords' circle its circle
    for (std::size_t i = 0; i < n;) {
        if (group[i] >= 0) {
            const auto& run = runs[static_cast<std::size_t>(group[i])];
            if (run.first == i) {
                const std::size_t count = run.second;
                const Vec2 a = outline[i], b = outline[(i + count) % n];
                const Real L = len[i], theta = std::fabs(turn[i]);
                const Real R = L / (2.0 * std::sin(theta * 0.5));
                const Real chord = (b - a).length();
                Real sag = R - std::sqrt(std::max(Real(0), R * R - chord * chord * 0.25));
                const Real total = theta * count;
                if (total > kPi) sag = 2.0 * R - sag;   // the long way round
                Edge arc;
                arc.kind = Edge::Kind::Arc;
                arc.a = a;
                arc.b = b;
                arc.bulge = turn[i] > 0 ? -sag : sag;   // a left-turning (convex) run bulges right, outward
                s.outer.push_back(arc);
                i += count;
                continue;
            }
            ++i;   // a chord inside a run that began before the outline's first vertex: its arc is emitted at the run's start
            continue;
        }
        Edge line;
        line.a = outline[i];
        line.b = outline[(i + 1) % n];
        s.outer.push_back(line);
        ++i;
    }
    return s;
}

}  // namespace engine
