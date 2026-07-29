// ---------------------------------------------------------------------------
// tools/lot_lab.cpp — THE LOT LAB: design lots and floorplans in 2D, top-down.
//
// A headless drafting table for `docs/lot-system-plan.md`. It builds real site
// plans and real floorplans out of the proposed L0/L1/L3 vocabulary and draws
// them as SVG — so the 2D half of the city can be DESIGNED and REVIEWED without
// a GPU, a viewer, or a Metal build. That is the point: the plan's riskiest
// layer (the 2D kernel) is also the one that needs no renderer to judge, and
// this is the loop that judges it.
//
// Build (from the repo root). The only dependencies are two engine translation
// units — which is itself the plan's claim, that the 2D machinery already
// exists and is merely unwired:
//
//   c++ -std=c++17 -O1 tools/lot_lab.cpp \
//       src/engine/procgen/city/polygon.cpp \
//       src/engine/procgen/city/road_offset.cpp -o /tmp/lot_lab
//
// Run:
//   /tmp/lot_lab              # writes lots.svg / plans.svg / shapes.svg to .
//   OUT=/tmp /tmp/lot_lab 12  # ...to /tmp, seed 12
//
// Pure, seeded and deterministic, like the procgen it prototypes: a sheet is
// reproducible from its seed.
// ---------------------------------------------------------------------------

#include "../src/engine/procgen/city/polygon.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <string>
#include <vector>

using engine::Poly2;
using engine::Real;
using engine::Vec2;

namespace lab {

constexpr Real kTau = 6.283185307179586;

// The deterministic xorshift the city pipeline uses, so a design only moves
// when its seed does.
struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 0x9e3779b9u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    Real unit() { return (next() & 0xffffff) / static_cast<Real>(0x1000000); }
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
};

// ===========================================================================
// L0 — REGION (the plan's `Shape2`): an outer loop plus holes. The one thing
// today's bare `Poly2` cannot represent — courtyards, atria, a plaza with a
// planting bed — and the reason every massing op below can compose freely.
// ===========================================================================

struct Region {
    Poly2 outer;
    std::vector<Poly2> holes;

    bool empty() const { return outer.size() < 3; }
    Real area() const {
        Real a = engine::area(outer);
        for (const Poly2& h : holes) a -= engine::area(h);
        return a;
    }
};

// --- Exact rectilinear boolean --------------------------------------------
//
// The workhorse. Architecture is overwhelmingly right-angled, and for that case
// an exact boolean beats any sampled kernel: coordinate-compress the input
// rectangles onto their own grid lines — so cell boundaries sit at REAL
// coordinates, with no sampling error — mark the inside cells, then trace the
// boundary. Walls come out perfectly crisp, holes fall out for free, and a
// union that splits into two pieces is handled naturally.
//
// This is the "exact" half of the plan's two-implementations-one-interface
// decision (lot-system-plan.md §3.3).

struct Rect {
    Real x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    Rect() = default;
    Rect(Real a, Real b, Real c, Real d)
        : x0(std::min(a, c)), y0(std::min(b, d)),
          x1(std::max(a, c)), y1(std::max(b, d)) {}
    bool contains(Real x, Real y) const {
        return x >= x0 && x <= x1 && y >= y0 && y <= y1;
    }
    Real width() const { return x1 - x0; }
    Real height() const { return y1 - y0; }
    Vec2 center() const { return Vec2((x0 + x1) * 0.5, (y0 + y1) * 0.5); }
};

// A CSG expression over axis-aligned rectangles: union of `add`, minus `sub`.
// Every plan-grammar op below is one push onto one of these two lists.
struct RectSet {
    std::vector<Rect> add, sub;
    void unite(const Rect& r) { add.push_back(r); }
    void subtract(const Rect& r) { sub.push_back(r); }
    bool inside(Real x, Real y) const {
        bool in = false;
        for (const Rect& r : add) if (r.contains(x, y)) { in = true; break; }
        if (!in) return false;
        for (const Rect& r : sub) if (r.contains(x, y)) return false;
        return true;
    }
};

// Drop vertices sitting on a straight run (the trace emits one per grid line).
static Poly2 mergeCollinear(const Poly2& p) {
    if (p.size() < 4) return p;
    Poly2 out;
    const std::size_t n = p.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& a = p[(i + n - 1) % n];
        const Vec2& b = p[i];
        const Vec2& c = p[(i + 1) % n];
        const Vec2 d0 = b - a, d1 = c - b;
        if (std::fabs(engine::cross(d0, d1)) > 1e-6) out.push_back(b);
    }
    return out.size() >= 3 ? out : p;
}

// Trace a RectSet into regions. Boundary edges are emitted with the interior on
// the LEFT, so outer rings come back CCW (positive area) and holes CW.
static std::vector<Region> traceRectSet(const RectSet& rs) {
    std::vector<Real> xs, ys;
    for (const Rect& r : rs.add) { xs.push_back(r.x0); xs.push_back(r.x1); ys.push_back(r.y0); ys.push_back(r.y1); }
    for (const Rect& r : rs.sub) { xs.push_back(r.x0); xs.push_back(r.x1); ys.push_back(r.y0); ys.push_back(r.y1); }
    if (xs.size() < 2) return {};
    auto tidy = [](std::vector<Real>& v) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end(),
                            [](Real a, Real b) { return std::fabs(a - b) < 1e-6; }),
                v.end());
    };
    tidy(xs); tidy(ys);
    const int nx = static_cast<int>(xs.size()) - 1;
    const int ny = static_cast<int>(ys.size()) - 1;
    if (nx < 1 || ny < 1) return {};

    std::vector<char> in(static_cast<std::size_t>(nx) * ny, 0);
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
            in[static_cast<std::size_t>(j) * nx + i] =
                rs.inside((xs[i] + xs[i + 1]) * 0.5, (ys[j] + ys[j + 1]) * 0.5) ? 1 : 0;
    auto solid = [&](int i, int j) {
        return i >= 0 && j >= 0 && i < nx && j < ny &&
               in[static_cast<std::size_t>(j) * nx + i] != 0;
    };

    std::vector<std::pair<Vec2, Vec2>> segs;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (!solid(i, j)) continue;
            if (!solid(i + 1, j))   // +x face runs +y: interior on the left
                segs.push_back({Vec2(xs[i + 1], ys[j]), Vec2(xs[i + 1], ys[j + 1])});
            if (!solid(i - 1, j))   // -x face runs -y
                segs.push_back({Vec2(xs[i], ys[j + 1]), Vec2(xs[i], ys[j])});
            if (!solid(i, j + 1))   // +y face runs -x
                segs.push_back({Vec2(xs[i + 1], ys[j + 1]), Vec2(xs[i], ys[j + 1])});
            if (!solid(i, j - 1))   // -y face runs +x
                segs.push_back({Vec2(xs[i], ys[j]), Vec2(xs[i + 1], ys[j])});
        }
    }
    if (segs.empty()) return {};

    auto key = [](const Vec2& v) {
        return std::make_pair(static_cast<long long>(std::llround(v.x * 1e5)),
                              static_cast<long long>(std::llround(v.y * 1e5)));
    };
    std::map<std::pair<long long, long long>, std::vector<std::size_t>> byStart;
    for (std::size_t i = 0; i < segs.size(); ++i) byStart[key(segs[i].first)].push_back(i);
    std::vector<char> used(segs.size(), 0);
    std::vector<Poly2> loops;
    for (std::size_t s = 0; s < segs.size(); ++s) {
        if (used[s]) continue;
        Poly2 loop;
        std::size_t cur = s;
        for (int guard = 0; guard < 200000; ++guard) {
            used[cur] = 1;
            loop.push_back(segs[cur].first);
            auto it = byStart.find(key(segs[cur].second));
            if (it == byStart.end()) break;
            std::size_t nxt = SIZE_MAX;
            for (std::size_t cand : it->second) if (!used[cand]) { nxt = cand; break; }
            if (nxt == SIZE_MAX) break;                  // loop closed
            cur = nxt;
        }
        if (loop.size() >= 4) loops.push_back(mergeCollinear(loop));
    }

    std::vector<Region> regions;
    std::vector<Poly2> holes;
    for (Poly2& l : loops) {
        if (engine::signedArea(l) > 0) { Region r; r.outer = l; regions.push_back(r); }
        else holes.push_back(l);
    }
    for (Poly2& h : holes) {              // nest each hole in its smallest container
        int best = -1;
        Real bestA = 1e30;
        for (std::size_t i = 0; i < regions.size(); ++i) {
            if (!engine::pointInPolygon(regions[i].outer, h[0])) continue;
            const Real a = engine::area(regions[i].outer);
            if (a < bestA) { bestA = a; best = static_cast<int>(i); }
        }
        if (best >= 0) regions[static_cast<std::size_t>(best)].holes.push_back(h);
    }
    std::sort(regions.begin(), regions.end(), [](const Region& a, const Region& b) {
        return engine::area(a.outer) > engine::area(b.outer);
    });
    return regions;
}

// --- Sampled-field path ---------------------------------------------------
//
// The robust complement (plan §3.3): booleans are min/max, offsets are an iso
// shift, rounding is offset-out-then-back — and topology changes come free.
// Used for the organic shapes the exact path can't express.

struct Field {
    Vec2 lo, hi;
    int nx = 0, ny = 0;
    Real cell = 0.1;
    std::vector<Real> d;
    Real at(int i, int j) const {
        i = std::max(0, std::min(nx - 1, i));
        j = std::max(0, std::min(ny - 1, j));
        return d[static_cast<std::size_t>(j) * nx + i];
    }
    Vec2 pos(int i, int j) const { return Vec2(lo.x + i * cell, lo.y + j * cell); }
};

static Field fieldMake(const Vec2& lo, const Vec2& hi, Real cell) {
    Field f;
    f.lo = lo; f.hi = hi; f.cell = cell;
    f.nx = static_cast<int>((hi.x - lo.x) / cell) + 2;
    f.ny = static_cast<int>((hi.y - lo.y) / cell) + 2;
    f.d.assign(static_cast<std::size_t>(f.nx) * f.ny, 1e9);
    return f;
}
enum class Op { Unite, Subtract, Intersect };
static void fieldApply(Field& f, Op op, const std::function<Real(const Vec2&)>& sdf) {
    for (int j = 0; j < f.ny; ++j)
        for (int i = 0; i < f.nx; ++i) {
            const Real dd = sdf(f.pos(i, j));
            Real& t = f.d[static_cast<std::size_t>(j) * f.nx + i];
            switch (op) {
                case Op::Unite:     t = std::min(t, dd); break;
                case Op::Subtract:  t = std::max(t, -dd); break;
                case Op::Intersect: t = std::max(t, dd); break;
            }
        }
}
static std::function<Real(const Vec2&)> sdfCircle(const Vec2& c, Real r) {
    return [c, r](const Vec2& p) { return (p - c).length() - r; };
}
static std::function<Real(const Vec2&)> sdfBox(const Rect& b) {
    const Vec2 c = b.center();
    const Vec2 h(b.width() * 0.5, b.height() * 0.5);
    return [c, h](const Vec2& p) {
        const Vec2 q(std::fabs(p.x - c.x) - h.x, std::fabs(p.y - c.y) - h.y);
        return Vec2(std::max(q.x, Real(0)), std::max(q.y, Real(0))).length() +
               std::min(std::max(q.x, q.y), Real(0));
    };
}

// Marching squares with edge interpolation — smooth contours on smooth fields.
static std::vector<Poly2> fieldContour(const Field& f, Real iso = 0.0) {
    std::vector<std::pair<Vec2, Vec2>> segs;
    auto lerpEdge = [&](const Vec2& pa, Real va, const Vec2& pb, Real vb) {
        const Real t = std::fabs(vb - va) < 1e-12 ? Real(0.5) : (iso - va) / (vb - va);
        return pa + (pb - pa) * std::max(Real(0), std::min(Real(1), t));
    };
    for (int j = 0; j < f.ny - 1; ++j) {
        for (int i = 0; i < f.nx - 1; ++i) {
            const Real v[4] = {f.at(i, j), f.at(i + 1, j), f.at(i + 1, j + 1), f.at(i, j + 1)};
            const Vec2 p[4] = {f.pos(i, j), f.pos(i + 1, j), f.pos(i + 1, j + 1), f.pos(i, j + 1)};
            int code = 0;
            for (int k = 0; k < 4; ++k) if (v[k] < iso) code |= (1 << k);
            if (code == 0 || code == 15) continue;
            std::vector<Vec2> hit;
            for (int k = 0; k < 4; ++k) {
                const int k2 = (k + 1) & 3;
                if ((v[k] < iso) != (v[k2] < iso))
                    hit.push_back(lerpEdge(p[k], v[k], p[k2], v[k2]));
            }
            for (std::size_t k = 0; k + 1 < hit.size(); k += 2)
                segs.push_back({hit[k], hit[k + 1]});
        }
    }
    std::vector<char> used(segs.size(), 0);
    std::vector<Poly2> loops;
    for (std::size_t s = 0; s < segs.size(); ++s) {
        if (used[s]) continue;
        Poly2 loop{segs[s].first, segs[s].second};
        used[s] = 1;
        for (int guard = 0; guard < 200000; ++guard) {
            const Vec2 tail = loop.back();
            std::size_t best = SIZE_MAX;
            Real bestD = f.cell * 1.6;
            bool flip = false;
            for (std::size_t t = 0; t < segs.size(); ++t) {
                if (used[t]) continue;
                const Real d0 = (segs[t].first - tail).length();
                const Real d1 = (segs[t].second - tail).length();
                if (d0 < bestD) { bestD = d0; best = t; flip = false; }
                if (d1 < bestD) { bestD = d1; best = t; flip = true; }
            }
            if (best == SIZE_MAX) break;
            used[best] = 1;
            loop.push_back(flip ? segs[best].first : segs[best].second);
        }
        if (loop.size() >= 8) loops.push_back(loop);
    }
    return loops;
}

// --- ARC EDGES: the mixed curved/straight loop -----------------------------
//
// The plan's `Shape2` edge, made real. A loop carries one BULGE per edge in the
// DXF convention — bulge = tan(sweep/4), 0 = straight, 1 = semicircle — so a
// wall is either a line or a TRUE circular arc, never a bezier standing in for
// one and never marching-squares wobble. Curves survive as curves through the
// grammar and tessellate exactly once, at the end, at a chord tolerance the
// caller picks (and the SVG writer doesn't tessellate them at all — it emits
// real `A` arc commands).
//
// This is what the first Lot Lab pass argued for: rounding a square with a
// quadratic bezier is visibly not a circle. With bulge it is one, to the
// numerical limit.

struct Arc2 {
    Poly2 pts;                  // loop vertices
    std::vector<Real> bulge;    // bulge[i] applies to the edge pts[i] -> pts[i+1]

    std::size_t size() const { return pts.size(); }
    Vec2 next(std::size_t i) const { return pts[(i + 1) % pts.size()]; }
    void setBulge(std::size_t i, Real b) { bulge[i % bulge.size()] = b; }
};

static Arc2 arcFromPoly(const Poly2& p) {
    Arc2 a;
    a.pts = p;
    a.bulge.assign(p.size(), 0.0);
    return a;
}

// Circle through an arc edge: centre, radius, start angle and signed sweep.
struct ArcGeom { Vec2 c; Real r = 0, a0 = 0, sweep = 0; bool straight = true; };

static ArcGeom arcGeom(const Vec2& A, const Vec2& B, Real bulge) {
    ArcGeom g;
    if (std::fabs(bulge) < 1e-9) return g;
    const Vec2 d = B - A;
    const Real c = d.length();
    if (c < 1e-9) return g;
    g.straight = false;
    g.sweep = 4.0 * std::atan(bulge);              // DXF: bulge = tan(sweep/4)
    g.r = c / (2.0 * std::sin(std::fabs(g.sweep) * 0.5));
    // Centre sits off the chord midpoint along the perpendicular; which side
    // depends on the bulge sign (positive = counter-clockwise sweep).
    const Vec2 m = (A + B) * 0.5;
    const Vec2 n = engine::normalize(Vec2(-d.y, d.x));       // LEFT of A->B
    const Real h = std::sqrt(std::max(Real(0), g.r * g.r - c * c * 0.25));
    const Real side = (std::fabs(g.sweep) > kTau * 0.5) ? -1.0 : 1.0;   // major arc
    // A positive (counter-clockwise) sweep puts the CENTRE on the left, so the
    // arc itself swells to the RIGHT of A->B — and on a CCW loop, right is
    // outside. Getting this sign wrong is invisible in the SVG (an `A` command
    // re-derives the centre from radius + flags) and only shows up when the
    // loop is tessellated.
    g.c = m + n * (h * side * (bulge > 0 ? 1.0 : -1.0));
    g.a0 = std::atan2(A.y - g.c.y, A.x - g.c.x);
    return g;
}

// Flatten to chords no further than `chordTol` from the true arc.
static Poly2 tessellate(const Arc2& a, Real chordTol = 0.12) {
    Poly2 out;
    for (std::size_t i = 0; i < a.size(); ++i) {
        out.push_back(a.pts[i]);
        const ArcGeom g = arcGeom(a.pts[i], a.next(i), a.bulge[i]);
        if (g.straight) continue;
        const Real maxStep = 2.0 * std::acos(std::max(Real(-1),
                                 std::min(Real(1), 1.0 - chordTol / g.r)));
        const int n = std::max(2, static_cast<int>(std::ceil(std::fabs(g.sweep) /
                                                             std::max(maxStep, Real(1e-3)))));
        for (int k = 1; k < n; ++k) {
            const Real t = g.a0 + g.sweep * (static_cast<Real>(k) / n);
            out.push_back(Vec2(g.c.x + g.r * std::cos(t), g.c.y + g.r * std::sin(t)));
        }
    }
    return out;
}

// Area including the circular segments the chords cut off.
static Real arcArea(const Arc2& a) {
    Real ar = engine::signedArea(a.pts);
    for (std::size_t i = 0; i < a.size(); ++i) {
        const ArcGeom g = arcGeom(a.pts[i], a.next(i), a.bulge[i]);
        if (g.straight) continue;
        ar += 0.5 * g.r * g.r * (g.sweep - std::sin(g.sweep));
    }
    return std::fabs(ar);
}

// BOW an edge into an arc of the given sagitta. The arc swells to the RIGHT of
// the edge direction, and a CCW loop keeps its interior on the left — so a
// positive sagitta bows OUTWARD and a negative one scoops in. A bay window, a
// bowed shopfront, an apse when |sagitta| reaches half the chord.
static void bow(Arc2& a, std::size_t edge, Real sagitta) {
    const std::size_t i = edge % a.size();
    const Real c = (a.next(i) - a.pts[i]).length();
    if (c < 1e-6) return;
    a.bulge[i] = 2.0 * sagitta / c;
}
// Turn an edge into a clean semicircular end (an apse / a stadium end).
static void apse(Arc2& a, std::size_t edge, Real sign = 1.0) {
    a.bulge[edge % a.size()] = sign;      // bulge 1 == half turn
}

// FILLET corners with a TRUE tangent arc: back off both edges by the tangent
// distance and join them with an arc of exactly radius r. `radii[i] <= 0`
// leaves that corner sharp — which is how one plan mixes crisp party walls
// with a rounded street corner.
static Arc2 filletCorners(const Arc2& in, const std::vector<Real>& radii) {
    const std::size_t n = in.size();
    if (n < 3 || radii.size() != n) return in;
    Arc2 out;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        const Real r = radii[i];
        // Only straight-to-straight corners are filleted here; an arc already
        // meeting a wall is tangent-continuous or deliberately not.
        if (r <= 1e-6 || std::fabs(in.bulge[prev]) > 1e-9 ||
            std::fabs(in.bulge[i]) > 1e-9) {
            out.pts.push_back(in.pts[i]);
            out.bulge.push_back(in.bulge[i]);
            continue;
        }
        const Vec2 P = in.pts[prev], V = in.pts[i], N = in.next(i);
        const Vec2 d0 = engine::normalize(V - P), d1 = engine::normalize(N - V);
        const Real crs = engine::cross(d0, d1);
        const Real phi = std::atan2(crs, engine::dot(d0, d1));   // deviation angle
        if (std::fabs(phi) < 1e-4 || std::fabs(phi) > kTau * 0.5 - 1e-4) {
            out.pts.push_back(V);
            out.bulge.push_back(in.bulge[i]);
            continue;
        }
        // Tangent set-back, clamped so neighbouring fillets can't overrun.
        Real t = r * std::fabs(std::tan(phi * 0.5));
        t = std::min(t, std::min((V - P).length(), (N - V).length()) * 0.48);
        out.pts.push_back(V - d0 * t);
        out.bulge.push_back(std::tan(phi * 0.25));   // the fillet arc itself
        out.pts.push_back(V + d1 * t);
        out.bulge.push_back(in.bulge[i]);            // the outgoing wall
    }
    return out;
}
static Arc2 filletAllCorners(const Arc2& in, Real r) {
    return filletCorners(in, std::vector<Real>(in.size(), r));
}

// --- Polygon helpers ------------------------------------------------------

// Per-edge inset: each edge moves inward by its OWN distance. This is what
// makes setbacks a PROGRAM property (front 7 m, side 3 m, rear 8 m) instead of
// today's single `lotSetback` number per district.
static Poly2 insetEdges(const Poly2& poly, const std::vector<Real>& dist) {
    const std::size_t n = poly.size();
    if (n < 3 || dist.size() != n) return {};
    Poly2 p = poly;
    if (engine::signedArea(p) < 0) std::reverse(p.begin(), p.end());
    struct Line { Vec2 pt, dir; };
    std::vector<Line> lines(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 d = engine::normalize(p[(i + 1) % n] - p[i]);
        const Vec2 inward(-d.y, d.x);          // interior is left of a CCW edge
        lines[i] = {p[i] + inward * dist[i], d};
    }
    Poly2 out(n);
    Real maxD = 0;
    for (Real d : dist) maxD = std::max(maxD, std::fabs(d));
    for (std::size_t i = 0; i < n; ++i) {
        const Line& l0 = lines[(i + n - 1) % n];
        const Line& l1 = lines[i];
        const Real den = engine::cross(l0.dir, l1.dir);
        if (std::fabs(den) < 1e-9) { out[i] = l1.pt; continue; }
        Vec2 q = l0.pt + l0.dir * (engine::cross(l1.pt - l0.pt, l1.dir) / den);
        // MITER LIMIT — without it the chord joints of a filleted/rounded ring
        // fly off and the offset grows spikes (the same failure `offsetPlan`
        // documents in shape_grammar.cpp). Fall back to the edge-normal point.
        if ((q - p[i]).length() > maxD * 4.0 + 0.5) q = l1.pt;
        out[i] = q;
    }
    return engine::area(out) > 1e-3 ? out : Poly2{};
}

// Fillet every vertex by radius r. The exact-path answer to "round a square":
// push r toward half the side and the square becomes a circle.
static Poly2 filletAll(const Poly2& poly, Real r, int segs = 8) {
    const std::size_t n = poly.size();
    if (n < 3 || r <= 1e-6) return poly;
    Poly2 out;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& a = poly[(i + n - 1) % n];
        const Vec2& b = poly[i];
        const Vec2& c = poly[(i + 1) % n];
        const Vec2 d0 = engine::normalize(b - a), d1 = engine::normalize(c - b);
        if (std::fabs(engine::cross(d0, d1)) < 1e-6) { out.push_back(b); continue; }
        // Clamp so neighbouring fillets can never overrun each other.
        const Real cut = std::min(r, std::min((b - a).length(), (c - b).length()) * 0.5);
        const Vec2 p0 = b - d0 * cut, p1 = b + d1 * cut;
        for (int k = 0; k <= segs; ++k) {            // quadratic bezier ~ the arc
            const Real t = static_cast<Real>(k) / segs, mt = 1 - t;
            out.push_back(p0 * (mt * mt) + b * (2 * mt * t) + p1 * (t * t));
        }
    }
    return out;
}

// Clip a polygon to a convex lot (Sutherland-Hodgman against each lot edge).
static Poly2 clipToLot(const Poly2& poly, const Poly2& lot) {
    Poly2 cur = poly;
    Poly2 L = lot;
    if (engine::signedArea(L) < 0) std::reverse(L.begin(), L.end());
    for (std::size_t i = 0; i < L.size() && !cur.empty(); ++i) {
        const Vec2& a = L[i];
        const Vec2& b = L[(i + 1) % L.size()];
        const Vec2 d = engine::normalize(b - a);
        const Vec2 outward(d.y, -d.x);           // right of a CCW edge = outside
        cur = engine::clipHalfPlane(cur, outward, engine::dot(outward, a));
    }
    return cur;
}

// A lot's local frame: origin at the street-edge midpoint, +x along the street,
// +y into the lot. Every program and plan is authored here, so a recipe reads
// "7 m back from the street" and never touches world coordinates.
struct Frame {
    Vec2 o{0, 0}, ex{1, 0}, ey{0, 1};
    Vec2 toWorld(Real x, Real y) const { return o + ex * x + ey * y; }
    Vec2 toWorld(const Vec2& p) const { return toWorld(p.x, p.y); }
    Poly2 toWorld(const Poly2& p) const {
        Poly2 q; q.reserve(p.size());
        for (const Vec2& v : p) q.push_back(toWorld(v));
        return q;
    }
    Region toWorld(const Region& r) const {
        Region o2;
        o2.outer = toWorld(r.outer);
        for (const Poly2& h : r.holes) o2.holes.push_back(toWorld(h));
        return o2;
    }
    Vec2 toLocal(const Vec2& w) const {
        const Vec2 d = w - o;
        return Vec2(engine::dot(d, ex), engine::dot(d, ey));
    }
};

static Poly2 rectPoly(const Rect& r) {
    return Poly2{Vec2(r.x0, r.y0), Vec2(r.x1, r.y0), Vec2(r.x1, r.y1), Vec2(r.x0, r.y1)};
}
static Region regionOf(const Poly2& p) { Region r; r.outer = p; return r; }

// ===========================================================================
// L3 — THE SITE PLAN. Zones tile the lot by SUBTRACTION, so overlap is
// impossible by construction rather than something rejection sampling must
// catch afterwards (lot-system-plan.md §8.2).
// ===========================================================================

enum class Zone : uint8_t { Building, Frontage, Circulation, Open, Parking, Service };

static const char* zoneName(Zone z) {
    switch (z) {
        case Zone::Building:    return "building";
        case Zone::Frontage:    return "frontage";
        case Zone::Circulation: return "circulation";
        case Zone::Open:        return "open";
        case Zone::Parking:     return "parking";
        default:                return "service";
    }
}
static const char* zoneFill(Zone z) {
    switch (z) {
        case Zone::Building:    return "#b9c0cc";
        case Zone::Frontage:    return "#ece2c6";
        case Zone::Circulation: return "#ddd6c8";
        case Zone::Open:        return "#cfe1c2";
        case Zone::Parking:     return "#d6d3cf";
        default:                return "#e0d7d7";
    }
}

enum class PropKind : uint8_t { Tree, Shrub, Planter, Bench, Lamp, Bollard, Bin, Car };

struct Prop {
    PropKind kind;
    Vec2 at;
    Real radius = 0.5;
    Real rot = 0;
};
struct FenceRun {
    Poly2 line;                // open polyline
    bool hedge = false;
};
struct Gate {
    Vec2 hinge, latch;
    Real swing = 1.0;          // which side the leaf swings toward
};

struct SitePlan {
    std::string program, note;
    Poly2 lot;
    Frame frame;
    std::vector<std::pair<Zone, Region>> zones;
    std::vector<Region> building;  // one plan may be SEVERAL masses
    std::vector<Poly2> tiers;      // upper massing outlines, drawn as dashed
    std::vector<Poly2> paths;      // circulation centrelines
    std::vector<FenceRun> fences;
    std::vector<Gate> gates;
    std::vector<Prop> props;
    std::vector<std::pair<Vec2, Vec2>> partyWalls;   // unit divisions in a terrace
    std::vector<Vec2> doors;                         // every entrance, not just one
    Vec2 entrance{0, 0};
    Real lotArea = 0, coverage = 0, lotW = 0, lotD = 0;
    int floors = 1;
};

static bool inRegion(const Region& r, const Vec2& p) {
    if (!engine::pointInPolygon(r.outer, p)) return false;
    for (const Poly2& h : r.holes) if (engine::pointInPolygon(h, p)) return false;
    return true;
}

// Dart-throwing with a spacing radius: reject anything outside its zone, too
// near the zone boundary, or overlapping something already placed. The same
// claim-your-footprint invariant as the zones, one level down.
static void scatterProps(std::vector<Prop>& out, const Region& zone, PropKind kind,
                         Real radius, int want, Rng& rng, Real edgeClear = 0.6) {
    if (zone.empty() || want <= 0) return;
    Vec2 lo, hi;
    engine::bounds(zone.outer, lo, hi);
    int placed = 0;
    for (int tries = 0; tries < want * 120 && placed < want; ++tries) {
        const Vec2 p(rng.range(lo.x, hi.x), rng.range(lo.y, hi.y));
        if (!inRegion(zone, p)) continue;
        bool nearEdge = false;
        auto checkRing = [&](const Poly2& ring) {
            for (std::size_t i = 0; i < ring.size() && !nearEdge; ++i) {
                const Vec2& a = ring[i];
                const Vec2& b = ring[(i + 1) % ring.size()];
                const Vec2 ab = b - a;
                const Real l2 = ab.lengthSquared();
                Real t = l2 > 1e-9 ? engine::dot(p - a, ab) / l2 : 0;
                t = std::max(Real(0), std::min(Real(1), t));
                if ((p - (a + ab * t)).length() < radius + edgeClear) nearEdge = true;
            }
        };
        checkRing(zone.outer);
        for (const Poly2& h : zone.holes) checkRing(h);
        if (nearEdge) continue;
        bool clash = false;
        for (const Prop& q : out)
            if ((q.at - p).length() < radius + q.radius + 0.5) { clash = true; break; }
        if (clash) continue;
        out.push_back({kind, p, radius, rng.range(0, kTau)});
        ++placed;
    }
}

// Place a gate where a path crosses a fence run: split the run and record the
// opening. The gate exists because the geometry demands one — not because a
// die said so (plan §8.2 step 6).
static void gateWherePathCrosses(std::vector<FenceRun>& fences, std::vector<Gate>& gates,
                                 const Poly2& path, Real width) {
    for (std::size_t fi = 0; fi < fences.size(); ++fi) {
        FenceRun& run = fences[fi];
        for (std::size_t i = 0; i + 1 < run.line.size(); ++i) {
            const Vec2 a = run.line[i], b = run.line[i + 1];
            for (std::size_t j = 0; j + 1 < path.size(); ++j) {
                const Vec2 c = path[j], d = path[j + 1];
                const Vec2 r = b - a, s = d - c;
                const Real den = engine::cross(r, s);
                if (std::fabs(den) < 1e-9) continue;
                const Real t = engine::cross(c - a, s) / den;
                const Real u = engine::cross(c - a, r) / den;
                if (t < 0 || t > 1 || u < 0 || u > 1) continue;
                const Vec2 hit = a + r * t;
                const Vec2 dir = engine::normalize(r);
                const Vec2 h0 = hit - dir * (width * 0.5);
                const Vec2 h1 = hit + dir * (width * 0.5);
                FenceRun tail;
                tail.hedge = run.hedge;
                tail.line.push_back(h1);
                for (std::size_t k = i + 1; k < run.line.size(); ++k)
                    tail.line.push_back(run.line[k]);
                run.line.resize(i + 1);
                run.line.push_back(h0);
                gates.push_back({h0, h1, 1.0});
                if (tail.line.size() >= 2) fences.push_back(tail);
                return;
            }
        }
    }
}

// ===========================================================================
// L1a — THE PLAN GRAMMAR. Ops over a RectSet in the plan's own frame: the
// brief's "start with a box, iterate each edge, outset, grow wings, inset the
// back face". Every plan is clipped to its envelope at the end, so it can
// never leave its zone (plan §4).
// ===========================================================================

struct PlanCtx {
    RectSet rs;
    Rect envelope;     // the reserved zone; nothing may leave it
    Rect base;         // the seed mass ops push off (must sit inside `envelope`)
};

// Push a bay / limb out from one side of the CURRENT base mass. Side 0 = the
// street edge, then clockwise. `a0..a1` runs along that edge from its start.
static void outset(PlanCtx& c, int side, Real a0, Real a1, Real depth) {
    const Rect& b = c.base;
    switch (side) {
        case 0: c.rs.unite(Rect(b.x0 + a0, b.y0 - depth, b.x0 + a1, b.y0 + 0.02)); break;
        case 1: c.rs.unite(Rect(b.x1 - 0.02, b.y0 + a0, b.x1 + depth, b.y0 + a1)); break;
        case 2: c.rs.unite(Rect(b.x0 + a0, b.y1 - 0.02, b.x0 + a1, b.y1 + depth)); break;
        default: c.rs.unite(Rect(b.x0 - depth, b.y0 + a0, b.x0 + 0.02, b.y0 + a1)); break;
    }
}
// Seed the base mass (and record it, so ops have an edge to push off).
static void seedMass(PlanCtx& c, const Rect& r) { c.base = r; c.rs.unite(r); }
// A bite out of the mass — the L / U / courtyard maker.
static void court(PlanCtx& c, Real x0, Real y0, Real x1, Real y1) {
    c.rs.subtract(Rect(x0, y0, x1, y1));
}
// Clip to the envelope and trace: the conformance guarantee. Returns EVERY
// piece — a plan is legitimately more than one mass (an office park's two
// blocks, twin towers standing clear of each other at grade), and keeping only
// the largest is exactly the assumption this system exists to break.
static std::vector<Region> planFinish(const PlanCtx& c, bool clip = true) {
    RectSet out;
    for (const Rect& r : c.rs.add) {
        if (!clip) { out.add.push_back(r); continue; }
        const Rect q(std::max(r.x0, c.envelope.x0), std::max(r.y0, c.envelope.y0),
                     std::min(r.x1, c.envelope.x1), std::min(r.y1, c.envelope.y1));
        if (q.width() > 0.05 && q.height() > 0.05) out.add.push_back(q);
    }
    out.sub = c.rs.sub;
    return traceRectSet(out);
}

// ===========================================================================
// SVG output
// ===========================================================================

struct Svg {
    FILE* f = nullptr;
    Real ox = 0, oy = 0, scale = 6.0;
    Real X(Real wx) const { return ox + wx * scale; }
    Real Y(Real wy) const { return oy + wy * scale; }

    void poly(const Poly2& p, const char* fill, const char* stroke, Real sw,
              const char* extra = "") {
        if (p.size() < 2) return;
        fprintf(f, "<path d=\"");
        for (std::size_t i = 0; i < p.size(); ++i)
            fprintf(f, "%c%.2f %.2f ", i ? 'L' : 'M', X(p[i].x), Y(p[i].y));
        fprintf(f, "Z\" fill=\"%s\" stroke=\"%s\" stroke-width=\"%.2f\" %s/>\n",
                fill, stroke, sw, extra);
    }
    void region(const Region& r, const char* fill, const char* stroke, Real sw,
                const char* extra = "") {
        if (r.outer.size() < 3) return;
        fprintf(f, "<path fill-rule=\"evenodd\" d=\"");
        auto ring = [&](const Poly2& p) {
            for (std::size_t i = 0; i < p.size(); ++i)
                fprintf(f, "%c%.2f %.2f ", i ? 'L' : 'M', X(p[i].x), Y(p[i].y));
            fprintf(f, "Z ");
        };
        ring(r.outer);
        for (const Poly2& h : r.holes) ring(h);
        fprintf(f, "\" fill=\"%s\" stroke=\"%s\" stroke-width=\"%.2f\" %s/>\n",
                fill, stroke, sw, extra);
    }
    void polyline(const Poly2& p, const char* stroke, Real sw, const char* extra = "") {
        if (p.size() < 2) return;
        fprintf(f, "<path fill=\"none\" d=\"");
        for (std::size_t i = 0; i < p.size(); ++i)
            fprintf(f, "%c%.2f %.2f ", i ? 'L' : 'M', X(p[i].x), Y(p[i].y));
        fprintf(f, "\" stroke=\"%s\" stroke-width=\"%.2f\" %s/>\n", stroke, sw, extra);
    }
    void line(const Vec2& a, const Vec2& b, const char* stroke, Real sw,
              const char* extra = "") {
        fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"%s\" "
                   "stroke-width=\"%.2f\" %s/>\n", X(a.x), Y(a.y), X(b.x), Y(b.y),
                stroke, sw, extra);
    }
    void circle(const Vec2& c, Real r, const char* fill, const char* stroke, Real sw,
                const char* extra = "") {
        fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"%s\" stroke=\"%s\" "
                   "stroke-width=\"%.2f\" %s/>\n", X(c.x), Y(c.y), r * scale, fill, stroke,
                sw, extra);
    }
    // An arc loop drawn with REAL SVG arc commands — no tessellation on the way
    // out, so a curved wall is a curve in the file, not a 200-point polyline.
    void arcLoop(const Arc2& a, const char* fill, const char* stroke, Real sw,
                 const char* extra = "") {
        if (a.size() < 3) return;
        fprintf(f, "<path d=\"M%.2f %.2f ", X(a.pts[0].x), Y(a.pts[0].y));
        for (std::size_t i = 0; i < a.size(); ++i) {
            const Vec2 nx = a.next(i);
            const ArcGeom g = arcGeom(a.pts[i], nx, a.bulge[i]);
            if (g.straight) {
                fprintf(f, "L%.2f %.2f ", X(nx.x), Y(nx.y));
            } else {
                const Real rr = g.r * scale;
                const int large = std::fabs(g.sweep) > kTau * 0.5 ? 1 : 0;
                const int sweep = g.sweep > 0 ? 1 : 0;   // our map preserves handedness
                fprintf(f, "A%.2f %.2f 0 %d %d %.2f %.2f ", rr, rr, large, sweep,
                        X(nx.x), Y(nx.y));
            }
        }
        fprintf(f, "Z\" fill=\"%s\" stroke=\"%s\" stroke-width=\"%.2f\" %s/>\n",
                fill, stroke, sw, extra);
    }
    // A quarter-circle swing arc — the thing that makes a gate read as a gate.
    void arc(const Vec2& c, const Vec2& from, Real sweepDeg, const char* stroke, Real sw) {
        const Vec2 v = from - c;
        const Real r = v.length();
        const Real a0 = std::atan2(v.y, v.x);
        const Real a1 = a0 + sweepDeg * kTau / 360.0;
        const Vec2 e(c.x + r * std::cos(a1), c.y + r * std::sin(a1));
        fprintf(f, "<path fill=\"none\" d=\"M%.2f %.2f A %.2f %.2f 0 0 %d %.2f %.2f\" "
                   "stroke=\"%s\" stroke-width=\"%.2f\" stroke-dasharray=\"3 3\"/>\n",
                X(from.x), Y(from.y), r * scale, r * scale, sweepDeg > 0 ? 1 : 0,
                X(e.x), Y(e.y), stroke, sw);
    }
    void textPx(Real px, Real py, const char* s, Real size, const char* fill,
                const char* anchor = "start", const char* weight = "400") {
        fprintf(f, "<text x=\"%.1f\" y=\"%.1f\" font-family=\"Helvetica,Arial,sans-serif\" "
                   "font-size=\"%.1f\" fill=\"%s\" text-anchor=\"%s\" font-weight=\"%s\">"
                   "%s</text>\n", px, py, size, fill, anchor, weight, s);
    }
};

static void svgOpen(Svg& s, const char* path, Real w, Real h, const char* title) {
    s.f = fopen(path, "w");
    fprintf(s.f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(s.f, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" "
                 "width=\"%.0f\" height=\"%.0f\" viewBox=\"0 0 %.0f %.0f\">\n", w, h, w, h);
    fprintf(s.f, "<rect width=\"%.0f\" height=\"%.0f\" fill=\"#faf8f3\"/>\n", w, h);
    s.textPx(28, 44, title, 21, "#1d2430", "start", "600");
}
static void svgClose(Svg& s) { fprintf(s.f, "</svg>\n"); fclose(s.f); s.f = nullptr; }

// Draw everything a finished site plan carries.
static void drawSite(Svg& s, const SitePlan& sp, Real labelY) {
    // Street band along the frontage, so "which way is the street" is never
    // ambiguous — every design decision below hangs off it.
    {
        const Real w = sp.lotW;
        Poly2 band{sp.frame.toWorld(-3, -7.0), sp.frame.toWorld(-3, -0.15),
                   sp.frame.toWorld(w + 3, -0.15), sp.frame.toWorld(w + 3, -7.0)};
        s.poly(band, "#e6e3dd", "none", 0);
        s.line(sp.frame.toWorld(-3, -3.6), sp.frame.toWorld(w + 3, -3.6),
               "#b9b4a8", 1.0, "stroke-dasharray=\"10 8\"");
        s.textPx(s.X(sp.frame.toWorld(w * 0.5, -5.2).x),
                 s.Y(sp.frame.toWorld(w * 0.5, -5.2).y), "STREET", 9,
                 "#a9a496", "middle", "600");
    }
    s.poly(sp.lot, "#f2eee5", "#8a8578", 1.2, "stroke-dasharray=\"7 4\"");
    for (const auto& [z, r] : sp.zones)
        if (z != Zone::Building) s.region(r, zoneFill(z), "#000000", 0.6, "stroke-opacity=\"0.10\"");
    for (const Poly2& p : sp.paths) s.polyline(p, "#cfc7b4", 9.0, "stroke-linejoin=\"round\" stroke-linecap=\"round\"");
    for (const Poly2& p : sp.paths) s.polyline(p, "#e5dece", 7.0, "stroke-linejoin=\"round\" stroke-linecap=\"round\"");

    for (const FenceRun& fr : sp.fences) {
        if (fr.hedge) {
            s.polyline(fr.line, "#7fa36a", 4.5, "stroke-linecap=\"round\"");
        } else {
            s.polyline(fr.line, "#6f6558", 1.6);
            for (std::size_t i = 0; i + 1 < fr.line.size(); ++i) {   // posts
                const Vec2 a = fr.line[i], b = fr.line[i + 1];
                const Real len = (b - a).length();
                const int n = std::max(1, static_cast<int>(len / 2.2));
                for (int k = 0; k <= n; ++k)
                    s.circle(a + (b - a) * (static_cast<Real>(k) / n), 0.16, "#6f6558", "none", 0);
            }
        }
    }
    for (const Gate& g : sp.gates) {
        const Vec2 dir = engine::normalize(g.latch - g.hinge);
        const Vec2 open(-dir.y * g.swing, dir.x * g.swing);
        const Real w = (g.latch - g.hinge).length();
        s.line(g.hinge, g.hinge + open * w, "#8c5a2b", 2.4);        // the leaf, swung
        s.arc(g.hinge, g.latch, 90 * g.swing, "#b08a5f", 1.0);      // its swing
        s.circle(g.hinge, 0.22, "#8c5a2b", "none", 0);              // the hinge
    }

    // Ground-floor plan heavy, then the upper tiers dashed on top of it — the
    // mass stack read as a plan, which is how an architect draws a setback.
    for (const Region& b : sp.building) s.region(b, "#b9c0cc", "#232a36", 2.0);
    for (const auto& [a, b] : sp.partyWalls) s.line(a, b, "#5a6272", 1.0);
    for (const Poly2& t : sp.tiers)
        s.poly(t, "none", "#414a5a", 1.1, "stroke-dasharray=\"5 4\"");

    for (const Prop& p : sp.props) {
        switch (p.kind) {
            case PropKind::Tree:
                s.circle(p.at, p.radius, "#8fb87a", "#5f8a4c", 0.8, "fill-opacity=\"0.55\"");
                s.circle(p.at, 0.22, "#6b5030", "none", 0); break;
            case PropKind::Shrub:  s.circle(p.at, p.radius, "#a3c48e", "#6d9257", 0.6); break;
            case PropKind::Planter:s.poly(rectPoly(Rect(p.at.x - p.radius, p.at.y - p.radius,
                                                        p.at.x + p.radius, p.at.y + p.radius)),
                                          "#c8b89a", "#8a7a5e", 0.8); break;
            case PropKind::Bench:  s.poly(rectPoly(Rect(p.at.x - 0.9, p.at.y - 0.28,
                                                        p.at.x + 0.9, p.at.y + 0.28)),
                                          "#a9906d", "#7a6247", 0.8); break;
            case PropKind::Lamp:
                s.circle(p.at, 2.6, "#ffd88a", "none", 0, "fill-opacity=\"0.16\"");
                s.circle(p.at, 0.30, "#3d4450", "#1d2430", 0.6); break;
            case PropKind::Bollard:s.circle(p.at, 0.20, "#5c6373", "none", 0); break;
            case PropKind::Bin:    s.circle(p.at, 0.45, "#8b8f96", "#5c6373", 0.6); break;
            case PropKind::Car:    s.poly(rectPoly(Rect(p.at.x - 0.9, p.at.y - 2.2,
                                                        p.at.x + 0.9, p.at.y + 2.2)),
                                          "#aab2c0", "#79808e", 0.8); break;
        }
    }
    for (const Vec2& d : sp.doors) s.circle(d, 0.40, "#d8452f", "#ffffff", 1.1);
    s.circle(sp.entrance, 0.42, "#d8452f", "#ffffff", 1.2);   // the main door

    char buf[256];
    const Real lx = s.X(0) - 8;
    snprintf(buf, sizeof buf, "%s", sp.program.c_str());
    s.textPx(lx, labelY, buf, 14.5, "#1d2430", "start", "600");
    snprintf(buf, sizeof buf, "%.0f m2 lot  ·  %.0f%% coverage  ·  %d floors",
             sp.lotArea, sp.coverage * 100.0, sp.floors);
    s.textPx(lx, labelY + 17, buf, 11.5, "#4b5563");
    snprintf(buf, sizeof buf, "%s", sp.note.c_str());
    s.textPx(lx, labelY + 32, buf, 10.5, "#9aa1ad");
}

// ---------------------------------------------------------------------------
// The programs and their site plans.
// ---------------------------------------------------------------------------

struct Program {
    const char* name;
    Real lotW, lotD;
    Real frontSet, sideSet, rearSet;
    Real coverage;
    int floors;
    const char* note;
};

// Build one site plan. Zones are carved IN ORDER and each is subtracted from
// what remains, which is the whole structural claim: nothing can overlap.
static SitePlan buildSite(const Program& pg, uint32_t seed, const Vec2& origin) {
    Rng rng(seed);
    SitePlan sp;
    sp.program = pg.name;
    sp.note = pg.note;
    sp.floors = pg.floors;
    sp.frame.o = origin;
    sp.frame.ex = Vec2(1, 0);
    sp.frame.ey = Vec2(0, 1);

    const std::string nm = pg.name;
    const bool flatiron = nm.rfind("flatiron", 0) == 0;

    // --- the parcel -------------------------------------------------------
    Poly2 lotL;
    if (flatiron) {                     // an acute corner between two streets
        lotL = Poly2{Vec2(0, 0), Vec2(pg.lotW, pg.lotD * 0.62),
                     Vec2(pg.lotW * 0.42, pg.lotD)};
    } else {
        lotL = rectPoly(Rect(0, 0, pg.lotW, pg.lotD));
    }
    sp.lot = sp.frame.toWorld(lotL);
    sp.lotArea = engine::area(lotL);
    sp.lotW = pg.lotW;
    sp.lotD = pg.lotD;

    // --- 1. setbacks -> the buildable envelope ---------------------------
    std::vector<Real> dist(lotL.size(), pg.sideSet);
    dist[0] = pg.frontSet;                              // edge 0 faces the street
    if (!flatiron) dist[2] = pg.rearSet;
    Poly2 envL = insetEdges(lotL, dist);
    if (envL.empty()) envL = lotL;
    const engine::OBB2 eb = engine::orientedBoundingBox(envL);
    Vec2 elo, ehi;
    engine::bounds(envL, elo, ehi);
    (void)eb;

    // --- 2. the building plan (L1a) --------------------------------------
    PlanCtx pc;
    pc.envelope = Rect(elo.x, elo.y, ehi.x, ehi.y);
    std::vector<Region> planL;

    if (flatiron) {
        // The prow lot: the plan IS the setback polygon with its acute nose
        // rounded — the classic flatiron, and the one case where the lot
        // legitimately drives the shape.
        planL = {regionOf(tessellate(filletAllCorners(arcFromPoly(envL), 3.4), 0.06))};
        sp.entrance = sp.frame.toWorld((envL[0] + envL[1]) * 0.5 + Vec2(0, 0.6));
    } else if (nm.rfind("villa", 0) == 0) {
        // A house is a small pad in a big lot: main block + a front bay + a
        // garage wing, leaving a garden front and back.
        const Real w = std::min(ehi.x - elo.x, Real(13.5));
        const Real d = std::min(ehi.y - elo.y, Real(11.0));
        const Real x0 = elo.x + 0.6, y0 = elo.y;
        pc.rs.unite(Rect(x0, y0, x0 + w, y0 + d));                    // main block
        pc.rs.unite(Rect(x0 + 1.6, y0 - 1.6, x0 + 6.2, y0 + 0.1));    // front bay
        pc.rs.unite(Rect(x0 + w - 0.1, y0 + 1.2, x0 + w + 5.6, y0 + 7.4));  // garage wing
        pc.envelope = Rect(elo.x, elo.y - 2.0, ehi.x, ehi.y);
        planL = planFinish(pc);
        sp.entrance = sp.frame.toWorld(x0 + 8.6, y0 - 0.1);
    } else if (nm.rfind("shopfront", 0) == 0) {
        // Old town: the building meets the pavement and fills its party walls;
        // a light well is carved out of the back.
        pc.rs.unite(Rect(elo.x, elo.y, ehi.x, ehi.y - 4.5));
        court(pc, elo.x + 2.6, ehi.y - 9.0, ehi.x - 2.6, ehi.y - 6.2);  // light well
        planL = planFinish(pc);
        sp.entrance = sp.frame.toWorld((elo.x + ehi.x) * 0.5, elo.y);
    } else if (nm.rfind("tower", 0) == 0) {
        // A tower is NOT lot-shaped: a smaller footprint held back from the
        // street so the frontage can be a real plaza, with chamfered corners.
        const Real cw = 25.0, cd = 24.0;
        const Real cx = (elo.x + ehi.x) * 0.5;
        const Real y0 = elo.y + 4.0;
        pc.rs.unite(Rect(cx - cw * 0.5, y0, cx + cw * 0.5, y0 + cd));
        pc.envelope = Rect(elo.x, elo.y, ehi.x, ehi.y);
        planL = planFinish(pc);
        // Upper tiers: each an independent plan constrained inside the one
        // below — the mass stack, not a uniform inset (plan §5). Inset the
        // ORTHOGONAL ring, then round each tier on its own radius; insetting an
        // already-rounded ring just feeds the miter limit chord joints.
        const Poly2 rect = planL.empty() ? envL : planL[0].outer;
        auto round = [](const Poly2& p, Real r) {
            return tessellate(filletAllCorners(arcFromPoly(p), r), 0.06);
        };
        Poly2 shaft = insetEdges(rect, std::vector<Real>(rect.size(), 2.2));
        if (!shaft.empty()) {
            sp.tiers.push_back(sp.frame.toWorld(round(shaft, 2.4)));
            Poly2 crown = insetEdges(shaft, std::vector<Real>(shaft.size(), 3.4));
            if (!crown.empty()) sp.tiers.push_back(sp.frame.toWorld(round(crown, 1.8)));
        }
        if (!planL.empty()) planL[0].outer = round(rect, 3.0);
        sp.entrance = sp.frame.toWorld(cx, y0);
    } else if (nm.rfind("rowhouse", 0) == 0) {
        // A terrace: ONE mass, read as units. Each unit gets its own front
        // door, its own stoop off the shared walk, and a party wall between —
        // which is what today's RowStrip massing can only do inside a lot it
        // owns outright.
        const Real y0 = elo.y + 3.0, y1 = elo.y + 13.0;
        seedMass(pc, Rect(elo.x, y0, ehi.x, y1));
        const int units = 6;
        const Real uw = (ehi.x - elo.x) / units;
        for (int u = 0; u < units; ++u) {          // alternating projecting bays
            if (u % 2 == 0) continue;
            pc.rs.unite(Rect(elo.x + uw * u + 0.6, y0 - 1.3,
                             elo.x + uw * (u + 1) - 0.6, y0 + 0.05));
        }
        pc.envelope = Rect(elo.x, y0 - 1.6, ehi.x, ehi.y);
        planL = planFinish(pc);
        for (int u = 1; u < units; ++u) {
            const Real x = elo.x + uw * u;
            sp.partyWalls.push_back({sp.frame.toWorld(x, y0), sp.frame.toWorld(x, y1)});
        }
        for (int u = 0; u < units; ++u) {
            const Real x = elo.x + uw * (u + 0.5);
            sp.doors.push_back(sp.frame.toWorld(x, u % 2 ? y0 - 1.3 : y0));
        }
        sp.entrance = sp.frame.toWorld(elo.x + uw * 0.5, y0);
    } else {
        // Office park: two blocks on a merged parcel, sharing a car park.
        pc.rs.unite(Rect(elo.x, elo.y + 2.0, elo.x + 26.0, elo.y + 19.0));
        pc.rs.unite(Rect(elo.x + 32.0, elo.y + 2.0, elo.x + 56.0, elo.y + 15.0));
        court(pc, elo.x + 8.0, elo.y + 8.0, elo.x + 18.0, elo.y + 19.5);  // atrium notch
        pc.envelope = Rect(elo.x, elo.y, ehi.x, ehi.y);
        planL = planFinish(pc);
        sp.entrance = sp.frame.toWorld(elo.x + 13.0, elo.y + 2.0);
    }
    if (planL.empty()) planL = {regionOf(envL)};
    Real builtArea = 0;
    for (const Region& r : planL) {
        sp.building.push_back(sp.frame.toWorld(r));
        builtArea += r.area();
    }
    sp.coverage = builtArea / std::max(Real(1), sp.lotArea);

    // --- 3. zones, carved in order ---------------------------------------
    auto addZone = [&](Zone z, const Poly2& p) {
        Poly2 c = clipToLot(p, lotL);
        if (c.size() >= 3 && engine::area(c) > 0.8)
            sp.zones.push_back({z, sp.frame.toWorld(regionOf(c))});
    };
    for (const Region& b : sp.building) sp.zones.push_back({Zone::Building, b});

    Vec2 blo, bhi;                                  // union bounds of every mass
    engine::bounds(planL[0].outer, blo, bhi);
    for (const Region& r : planL) {
        Vec2 a, b;
        engine::bounds(r.outer, a, b);
        blo = Vec2(std::min(blo.x, a.x), std::min(blo.y, a.y));
        bhi = Vec2(std::max(bhi.x, b.x), std::max(bhi.y, b.y));
    }

    if (nm.rfind("villa", 0) == 0) {
        addZone(Zone::Frontage, rectPoly(Rect(0, 0, pg.lotW, blo.y - 0.4)));
        addZone(Zone::Open, rectPoly(Rect(0, bhi.y + 0.4, pg.lotW, pg.lotD)));
        addZone(Zone::Circulation, rectPoly(Rect(bhi.x - 5.6, 0, bhi.x, blo.y + 7.4)));
    } else if (nm.rfind("shopfront", 0) == 0) {
        addZone(Zone::Service, rectPoly(Rect(0, bhi.y + 0.2, pg.lotW, pg.lotD)));
    } else if (nm.rfind("tower", 0) == 0) {
        addZone(Zone::Frontage, rectPoly(Rect(0, 0, pg.lotW, blo.y - 0.4)));
        addZone(Zone::Open, rectPoly(Rect(0, bhi.y + 0.4, pg.lotW, pg.lotD)));
        addZone(Zone::Circulation, rectPoly(Rect(0, blo.y - 0.4, elo.x, bhi.y + 0.4)));
        addZone(Zone::Service, rectPoly(Rect(ehi.x, blo.y, pg.lotW, bhi.y)));
    } else if (nm.rfind("rowhouse", 0) == 0) {
        addZone(Zone::Frontage, rectPoly(Rect(0, 0, pg.lotW, blo.y - 0.3)));
        addZone(Zone::Open, rectPoly(Rect(0, bhi.y + 0.3, pg.lotW, pg.lotD)));
    } else if (nm.rfind("office", 0) == 0) {
        addZone(Zone::Parking, rectPoly(Rect(0, bhi.y + 1.0, pg.lotW, pg.lotD - 2.0)));
        addZone(Zone::Frontage, rectPoly(Rect(0, 0, pg.lotW, blo.y - 0.5)));
        addZone(Zone::Open, rectPoly(Rect(elo.x + 26.5, elo.y + 2.0, elo.x + 31.5, bhi.y)));
    } else {
        addZone(Zone::Frontage, rectPoly(Rect(0, 0, pg.lotW, blo.y - 0.4)));
    }

    auto zoneOf = [&](Zone z) -> const Region* {
        for (const auto& [k, r] : sp.zones) if (k == z) return &r;
        return nullptr;
    };

    // --- 4. circulation: a path from the street to every door ------------
    if (!sp.doors.empty()) {
        for (const Vec2& dw : sp.doors) {
            const Vec2 dl = sp.frame.toLocal(dw);
            sp.paths.push_back(sp.frame.toWorld(Poly2{Vec2(dl.x, -0.2), Vec2(dl.x, dl.y)}));
        }
    } else {
        const Vec2 doorL = sp.frame.toLocal(sp.entrance);
        sp.paths.push_back(sp.frame.toWorld(Poly2{Vec2(doorL.x, -0.2), Vec2(doorL.x, doorL.y)}));
    }
    if (nm.rfind("villa", 0) == 0) {                       // driveway to the garage
        Poly2 drive{Vec2(bhi.x - 2.8, -0.2), Vec2(bhi.x - 2.8, blo.y + 4.4)};
        sp.paths.push_back(sp.frame.toWorld(drive));
    }
    if (nm.rfind("office", 0) == 0) {                      // car-park aisle
        Poly2 aisle{Vec2(pg.lotW - 8.0, -0.2), Vec2(pg.lotW - 8.0, pg.lotD - 6.0),
                    Vec2(6.0, pg.lotD - 6.0)};
        sp.paths.push_back(sp.frame.toWorld(aisle));
    }

    // --- 5. boundaries, then gates where a path crosses one --------------
    const bool fenced = nm.rfind("villa", 0) == 0 || nm.rfind("rowhouse", 0) == 0;
    if (fenced) {
        FenceRun front;
        front.line = sp.frame.toWorld(Poly2{Vec2(0.3, 0.35), Vec2(pg.lotW - 0.3, 0.35)});
        front.hedge = nm.rfind("rowhouse", 0) == 0;
        sp.fences.push_back(front);
        FenceRun side;
        side.line = sp.frame.toWorld(Poly2{Vec2(0.3, 0.35), Vec2(0.3, pg.lotD - 0.3),
                                           Vec2(pg.lotW - 0.3, pg.lotD - 0.3),
                                           Vec2(pg.lotW - 0.3, 0.35)});
        sp.fences.push_back(side);
        // A gate per crossing: six front walks through a hedge means six gates.
        for (std::size_t pi = 0; pi < sp.paths.size(); ++pi) {
            const bool drive = sp.paths[pi].size() > 2 ||
                               (nm.rfind("villa", 0) == 0 && pi + 1 == sp.paths.size());
            gateWherePathCrosses(sp.fences, sp.gates, sp.paths[pi], drive ? 3.2 : 1.3);
        }
    }

    // --- 6. furnish what's left ------------------------------------------
    if (const Region* fr = zoneOf(Zone::Frontage)) {
        scatterProps(sp.props, *fr, PropKind::Shrub, 0.65, nm.rfind("tower", 0) == 0 ? 0 : 5, rng);
        if (nm.rfind("tower", 0) == 0) {
            scatterProps(sp.props, *fr, PropKind::Planter, 1.1, 6, rng);
            scatterProps(sp.props, *fr, PropKind::Bench, 1.0, 5, rng);
            scatterProps(sp.props, *fr, PropKind::Tree, 2.2, 6, rng);
            scatterProps(sp.props, *fr, PropKind::Lamp, 0.35, 4, rng);
            scatterProps(sp.props, *fr, PropKind::Bollard, 0.25, 8, rng, 0.3);
        } else {
            scatterProps(sp.props, *fr, PropKind::Tree, 1.9, 2, rng);
            scatterProps(sp.props, *fr, PropKind::Lamp, 0.35, 1, rng);
        }
    }
    if (const Region* op = zoneOf(Zone::Open)) {
        scatterProps(sp.props, *op, PropKind::Tree, 2.4, 4, rng);
        scatterProps(sp.props, *op, PropKind::Shrub, 0.8, 6, rng);
        scatterProps(sp.props, *op, PropKind::Bench, 1.0, 1, rng);
    }
    if (const Region* pk = zoneOf(Zone::Parking)) {
        scatterProps(sp.props, *pk, PropKind::Car, 2.3, 14, rng, 0.4);
        scatterProps(sp.props, *pk, PropKind::Lamp, 0.35, 3, rng);
        scatterProps(sp.props, *pk, PropKind::Tree, 2.0, 3, rng);
    }
    if (const Region* sv = zoneOf(Zone::Service)) {
        scatterProps(sp.props, *sv, PropKind::Bin, 0.5, 3, rng, 0.4);
        scatterProps(sp.props, *sv, PropKind::Lamp, 0.35, 1, rng);
    }
    return sp;
}

// ---------------------------------------------------------------------------
// Sheet 1 — six site plans.
// ---------------------------------------------------------------------------
static void sheetLots(const char* dir, uint32_t seed) {
    static const Program kProgs[] = {
        {"villa — detached house, garden lot",     22, 34,  7.0, 3.0, 8.0, 0.22, 2,
         "garden, drive to garage, fenced rear, gate per crossing"},
        {"shopfront — old-town party-wall lot",    11, 26,  0.0, 0.0, 5.0, 0.62, 4,
         "meets the pavement; light well carved from the rear"},
        {"tower — corner plaza, financial core",   46, 50, 12.0, 6.0, 5.0, 0.26, 34,
         "held back from the street; frontage IS the plaza; 3 tiers"},
        {"rowhouse — terrace of six units",        34, 24,  4.0, 0.0, 7.0, 0.42, 3,
         "one mass, six doors, six gates; shared rear yard"},
        {"office park — merged parcel",            62, 46,  6.0, 4.0, 6.0, 0.24, 4,
         "two blocks, atrium notch, car park with an aisle"},
        {"flatiron — acute corner parcel",         30, 30,  2.0, 2.0, 2.0, 0.55, 8,
         "the one case where the lot drives the shape; prow rounded"},
    };

    // ONE scale for every plan on the sheet, the way a real drawing set works —
    // so a tower's parcel visibly IS four times a villa's. Variable lot size is
    // part of the proposal, not an incidental.
    const int n = static_cast<int>(sizeof kProgs / sizeof kProgs[0]);
    const Real SC = 4.3;
    Real maxW = 0, maxD = 0;
    for (int i = 0; i < n; ++i) {
        maxW = std::max(maxW, kProgs[i].lotW);
        maxD = std::max(maxD, kProgs[i].lotD);
    }
    const Real cellW = std::max(maxW * SC + 70, Real(390));
    const Real cellH = (maxD + 11) * SC + 108;
    const int cols = 3;
    const int rows = (n + cols - 1) / cols;
    Svg s;
    s.scale = SC;
    svgOpen(s, (std::string(dir) + "/lots.svg").c_str(), cellW * cols + 70,
            cellH * rows + 190, "Lot Lab — site plans (top-down, metres, one scale)");
    s.textPx(28, 68, "Zones are carved in order and subtracted from the remainder, so overlap is "
                     "impossible by construction. Red dot = door. Brown arc = gate swing. "
                     "Dashed inner outline = upper massing tier.", 12, "#6b7280");

    {   // one scale bar for the sheet
        fprintf(s.f, "<line x1=\"28\" y1=\"92\" x2=\"%.1f\" y2=\"92\" stroke=\"#a9a496\" "
                     "stroke-width=\"1.8\"/>\n", 28 + 20 * SC);
        s.textPx(28 + 20 * SC + 8, 96, "20 m", 10.5, "#a9a496");
    }
    for (int i = 0; i < n; ++i) {
        const int cx = i % cols, cy = i / cols;
        const Program& pg = kProgs[i];
        s.ox = 44 + cx * cellW + (maxW - pg.lotW) * SC * 0.5;
        s.oy = 118 + cy * cellH + 44;
        SitePlan sp = buildSite(pg, seed * 131u + static_cast<uint32_t>(i) * 17u + 3u,
                                Vec2(0, 0));
        drawSite(s, sp, s.oy + pg.lotD * SC + 30);
    }

    // legend
    const Real ly = 118 + rows * cellH + 30;
    const Zone zs[] = {Zone::Building, Zone::Frontage, Zone::Circulation,
                       Zone::Open, Zone::Parking, Zone::Service};
    for (int i = 0; i < 6; ++i) {
        const Real lx = 40 + i * 150;
        fprintf(s.f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"18\" height=\"12\" fill=\"%s\" "
                     "stroke=\"#000000\" stroke-opacity=\"0.14\"/>\n", lx, ly - 10, zoneFill(zs[i]));
        s.textPx(lx + 24, ly, zoneName(zs[i]), 11.5, "#4b5563");
    }
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 2 — the plan grammar, step by step.
// ---------------------------------------------------------------------------
static void sheetPlans(const char* dir, uint32_t seed) {
    struct Step { const char* title; const char* sub; };
    Svg s;
    const Real cellW = 340, cellH = 300;
    const int cols = 4, rows = 2;
    svgOpen(s, (std::string(dir) + "/plans.svg").c_str(), cellW * cols + 60,
            cellH * rows + 130, "Lot Lab — the plan grammar (L1a), step by step");
    s.textPx(28, 68, "One RectSet rewritten op by op, then traced. Exact rectilinear boolean: "
                     "crisp walls, holes for free, always clipped to the envelope.",
             12, "#6b7280");

    const Rect env(0, 0, 26, 20);
    struct Cell { const char* t; const char* d; PlanCtx pc; bool showEnv; };
    std::vector<Cell> cells;

    // The seed sits inside the envelope, so ops have room to push out and the
    // clip is a real constraint rather than a no-op.
    auto base = [&]() {
        PlanCtx c;
        c.envelope = env;
        seedMass(c, Rect(5, 4, 21, 16));
        return c;
    };

    cells.push_back({"1 · seed", "a box inside the envelope", base(), true});
    {   // 2 outset the street face
        PlanCtx pc = base();
        outset(pc, 0, 4.5, 11.5, 2.6);
        cells.push_back({"2 · outset", "a bay pushed out on the street edge", pc, true});
    }
    {   // 3 wings
        PlanCtx pc = base();
        outset(pc, 0, 4.5, 11.5, 2.6);
        outset(pc, 3, 0.5, 7.5, 3.6);
        outset(pc, 1, 0.5, 7.5, 3.6);
        cells.push_back({"3 · wings", "limbs off both side edges", pc, true});
    }
    {   // 4 court from the back -> a U
        PlanCtx pc = base();
        outset(pc, 0, 4.5, 11.5, 2.6);
        outset(pc, 3, 0.5, 7.5, 3.6);
        outset(pc, 1, 0.5, 7.5, 3.6);
        court(pc, 9, 10.5, 17, 16.5);
        cells.push_back({"4 · court", "a bite out of the rear: a U plan", pc, true});
    }
    {   // 5 courtyard -> a real hole
        PlanCtx pc = base();
        outset(pc, 0, 4.5, 11.5, 2.6);
        court(pc, 9.5, 7.5, 16.5, 12.5);
        cells.push_back({"5 · courtyard", "an interior hole — impossible today", pc, true});
    }
    {   // 6 two masses + link
        PlanCtx pc; pc.envelope = env;
        seedMass(pc, Rect(1.5, 3, 10, 17));
        pc.rs.unite(Rect(16, 3, 24.5, 17));
        pc.rs.unite(Rect(10, 8, 16, 12));
        cells.push_back({"6 · twin + link", "one plan, two towers, a bridging bar", pc, true});
    }
    {   // 7 pinwheel
        PlanCtx pc; pc.envelope = env;
        seedMass(pc, Rect(9, 7, 17, 13));
        outset(pc, 3, -1.0, 3.0, 7.0);
        outset(pc, 1, 3.0, 7.0, 7.0);
        outset(pc, 0, 1.5, 5.5, 6.0);
        outset(pc, 2, 2.5, 6.5, 6.0);
        cells.push_back({"7 · pinwheel", "four wings off a core", pc, true});
    }
    {   // 8 stepped
        PlanCtx pc; pc.envelope = env;
        seedMass(pc, Rect(2, 2, 24, 8));
        pc.rs.unite(Rect(4, 8, 22, 13));
        pc.rs.unite(Rect(7, 13, 19, 17));
        cells.push_back({"8 · stepped", "a base for a tiered mass stack", pc, true});
    }

    for (std::size_t i = 0; i < cells.size(); ++i) {
        const int cx = static_cast<int>(i) % cols, cy = static_cast<int>(i) / cols;
        s.scale = std::min((cellW - 70) / 30.0, (cellH - 110) / 24.0);
        s.ox = 40 + cx * cellW + 20;
        s.oy = 110 + cy * cellH + 20;
        if (cells[i].showEnv)
            s.poly(rectPoly(cells[i].pc.envelope), "#f2eee5", "#b9b4a8", 1.0,
                   "stroke-dasharray=\"6 4\"");
        std::vector<Region> rs = planFinish(cells[i].pc);
        Real ar = 0;
        int verts = 0, holes = 0;
        for (const Region& r : rs) {
            s.region(r, "#b9c0cc", "#232a36", 2.0);
            ar += r.area();
            verts += static_cast<int>(r.outer.size());
            holes += static_cast<int>(r.holes.size());
        }
        char buf[160];
        snprintf(buf, sizeof buf, "%s", cells[i].t);
        s.textPx(s.ox, s.oy + 24 * s.scale + 30, buf, 13.5, "#1d2430", "start", "600");
        snprintf(buf, sizeof buf, "%s", cells[i].d);
        s.textPx(s.ox, s.oy + 24 * s.scale + 47, buf, 11, "#6b7280");
        snprintf(buf, sizeof buf, "%.0f m2  ·  %d vertices  ·  %d hole(s)  ·  %d mass(es)",
                 ar, verts, holes, static_cast<int>(rs.size()));
        s.textPx(s.ox, s.oy + 24 * s.scale + 62, buf, 10, "#9aa1ad");
    }
    (void)seed;
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 4 — MIXED CURVED / STRAIGHT PLANS. Every one of these is a single loop
// where some edges are lines and some are true circular arcs. Straight walls
// stay crisp; curved walls are exact, not sampled and not bezier-faked.
// ---------------------------------------------------------------------------
static void sheetCurves(const char* dir, uint32_t seed) {
    struct Cell { const char* t; const char* d; Arc2 a; };
    std::vector<Cell> cells;

    auto ccw = [](Poly2 p) {
        if (engine::signedArea(p) < 0) std::reverse(p.begin(), p.end());
        return p;
    };

    {   // 1 — bay-front house: straight walls, one bowed bay, rounded front corners
        Arc2 a = arcFromPoly(ccw(Poly2{Vec2(0, 0), Vec2(4.5, 0), Vec2(9.5, 0),
                                       Vec2(14, 0), Vec2(14, 11), Vec2(0, 11)}));
        bow(a, 1, 1.7);                        // the bay swells toward the street
        std::vector<Real> r(a.size(), 0.0);
        r[0] = 0.9; r[3] = 0.9;                // soften only the street corners
        a = filletCorners(a, r);
        cells.push_back({"1 · bay front", "straight walls + one bowed bay", a});
    }
    {   // 2 — stadium/lozenge tower: two straight flanks, two semicircular ends
        Arc2 a = arcFromPoly(ccw(Poly2{Vec2(0, 0), Vec2(16, 0), Vec2(16, 10), Vec2(0, 10)}));
        apse(a, 1, 1.0);
        apse(a, 3, 1.0);
        cells.push_back({"2 · stadium plan", "flat flanks, semicircular ends", a});
    }
    {   // 3 — apsidal hall: a straight nave closed by a round apse
        Arc2 a = arcFromPoly(ccw(Poly2{Vec2(0, 0), Vec2(11, 0), Vec2(11, 20), Vec2(0, 20)}));
        apse(a, 2, 1.0);                       // the liturgical east end
        cells.push_back({"3 · apsidal hall", "nave straight, one round end", a});
    }
    {   // 4 — crescent terrace: concentric arcs joined by straight radial ends
        const Vec2 C(0, 0);
        const Real Ri = 22, Ro = 33, a0 = kTau * 0.06, a1 = kTau * 0.44;
        auto onC = [&](Real R, Real t) { return Vec2(C.x + R * std::cos(t), C.y + R * std::sin(t)); };
        Arc2 a;
        a.pts = {onC(Ri, a0), onC(Ro, a0), onC(Ro, a1), onC(Ri, a1)};
        a.bulge = {0, std::tan((a1 - a0) * 0.25), 0, -std::tan((a1 - a0) * 0.25)};
        cells.push_back({"4 · crescent", "two concentric arcs, radial ends", a});
    }
    {   // 5 — moderne slab: orthogonal, but the street corners turn a big radius
        Arc2 a = arcFromPoly(ccw(Poly2{Vec2(0, 0), Vec2(22, 0), Vec2(22, 12), Vec2(0, 12)}));
        std::vector<Real> r(a.size(), 0.0);
        r[0] = 4.2; r[1] = 4.2;                // only the two street corners
        a = filletCorners(a, r);
        cells.push_back({"5 · moderne slab", "sharp rear, 4.2 m radius street corners", a});
    }
    {   // 6 — bowed terrace: one mass, a bow window per unit (Bath / Boston)
        const int units = 5;
        const Real w = 30.0, uw = w / units;
        Poly2 p;
        for (int u = 0; u <= units; ++u) p.push_back(Vec2(uw * u, 0));
        p.push_back(Vec2(w, 12));
        p.push_back(Vec2(0, 12));
        Arc2 a = arcFromPoly(ccw(p));
        for (int u = 0; u < units; ++u) bow(a, static_cast<std::size_t>(u), 1.25);
        cells.push_back({"6 · bowed terrace", "a bow window per unit, shared party walls", a});
    }
    {   // 7 — flatiron with a TRUE arc prow (the bezier version was visibly not one)
        Arc2 a = arcFromPoly(ccw(Poly2{Vec2(0, 0), Vec2(26, 9), Vec2(9, 22)}));
        std::vector<Real> r(a.size(), 0.0);
        r[1] = 3.2;                            // the acute nose
        r[0] = 1.4; r[2] = 1.4;
        a = filletCorners(a, r);
        cells.push_back({"7 · flatiron prow", "acute corner rounded by a real arc", a});
    }
    {   // 8 — rotunda + wings: a drum and two straight bars, solved analytically
        const Real R = 9.0, hw = 4.0, xo = 22.0;
        const Real xi = std::sqrt(std::max(Real(0), R * R - hw * hw));
        const Real al = std::atan2(hw, xi);
        const Real sweep = kTau * 0.5 - 2 * al;
        Arc2 a;
        a.pts = {Vec2(xi, -hw), Vec2(xo, -hw), Vec2(xo, hw), Vec2(xi, hw),
                 Vec2(-xi, hw), Vec2(-xo, hw), Vec2(-xo, -hw), Vec2(-xi, -hw)};
        a.bulge.assign(8, 0.0);
        a.bulge[3] = std::tan(sweep * 0.25);   // over the top of the drum
        a.bulge[7] = std::tan(sweep * 0.25);   // under it
        cells.push_back({"8 · rotunda + wings", "circular drum, straight wings", a});
    }

    const Real cellW = 340, cellH = 290;
    const int cols = 4, rows = 2;
    Svg s;
    svgOpen(s, (std::string(dir) + "/curves.svg").c_str(), cellW * cols + 60,
            cellH * rows + 190, "Lot Lab — mixed curved / straight floorplans");
    s.textPx(28, 68, "One loop, per-edge bulge (DXF convention: bulge = tan(sweep/4)). Lines stay "
                     "lines, curves are TRUE circular arcs — and the SVG carries them as arc "
                     "commands, not polylines.", 12, "#6b7280");

    const Real drawW = cellW - 80, drawH = 190;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const int cx = static_cast<int>(i) % cols, cy = static_cast<int>(i) / cols;
        const Real bx = 40 + cx * cellW + 26;          // cell origin, px
        const Real by = 118 + cy * cellH;
        Vec2 lo, hi;
        engine::bounds(tessellate(cells[i].a, 0.05), lo, hi);
        const Real w = std::max(hi.x - lo.x, Real(1)), h = std::max(hi.y - lo.y, Real(1));
        s.scale = std::min(drawW / w, drawH / h);
        // Centre the plan in a fixed box so every label sits on one baseline.
        s.ox = bx + (drawW - w * s.scale) * 0.5 - lo.x * s.scale;
        s.oy = by + (drawH - h * s.scale) * 0.5 - lo.y * s.scale;
        s.arcLoop(cells[i].a, "#b9c0cc", "#232a36", 2.0);

        int arcs = 0;
        for (Real b : cells[i].a.bulge) if (std::fabs(b) > 1e-9) ++arcs;
        char buf[192];
        snprintf(buf, sizeof buf, "%s", cells[i].t);
        s.textPx(bx, by + drawH + 30, buf, 13.5, "#1d2430", "start", "600");
        snprintf(buf, sizeof buf, "%s", cells[i].d);
        s.textPx(bx, by + drawH + 47, buf, 11, "#6b7280");
        snprintf(buf, sizeof buf, "%.0f m2  ·  %d edges, %d curved",
                 arcArea(cells[i].a), static_cast<int>(cells[i].a.size()), arcs);
        s.textPx(bx, by + drawH + 62, buf, 10, "#9aa1ad");
    }
    (void)seed;
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 3 — the kernel: exact fillets, and the sampled-field boolean path.
// ---------------------------------------------------------------------------
static void sheetShapes(const char* dir, uint32_t seed) {
    Svg s;
    const Real cellW = 300, cellH = 290;
    svgOpen(s, (std::string(dir) + "/shapes.svg").c_str(), cellW * 4 + 60, cellH * 2 + 140,
            "Lot Lab — the 2D kernel (L0)");
    s.textPx(28, 68, "Top: exact per-vertex fillet, square rounded toward a circle. "
                     "Bottom: the sampled distance-field path — booleans as min/max, "
                     "offset as an iso shift.", 12, "#6b7280");

    // Row 1: square -> circle by fillet radius.
    const Poly2 sq = rectPoly(Rect(-8, -8, 8, 8));
    const Real radii[4] = {0.0, 2.5, 5.5, 8.0};
    for (int i = 0; i < 4; ++i) {
        s.scale = 7.6;
        s.ox = 40 + i * cellW + cellW * 0.5 - 20;
        s.oy = 130 + 90;
        s.poly(sq, "none", "#c8c2b4", 1.0, "stroke-dasharray=\"5 4\"");
        Poly2 r = filletAll(sq, radii[i], 10);
        s.poly(r, "#b9c0cc", "#232a36", 2.0);
        char buf[96];
        snprintf(buf, sizeof buf, "fillet r = %.1f m", radii[i]);
        s.textPx(s.ox - 60, s.oy + 78, buf, 12.5, "#1d2430", "start", "600");
        snprintf(buf, sizeof buf, "%d vertices", static_cast<int>(r.size()));
        s.textPx(s.ox - 60, s.oy + 94, buf, 10.5, "#9aa1ad");
    }

    // Row 2: the field path — union / subtract / intersect / offset.
    struct FCase { const char* t; const char* d; int mode; };
    const FCase cases[4] = {
        {"union", "two discs, min()", 0},
        {"subtract", "disc minus disc, max(-d)", 1},
        {"intersect", "lens, max()", 2},
        {"offset", "box+disc, iso shifted out 1.5 m", 3},
    };
    for (int i = 0; i < 4; ++i) {
        s.scale = 7.6;
        s.ox = 40 + i * cellW + cellW * 0.5 - 20;
        s.oy = 130 + cellH + 90;
        Field f = fieldMake(Vec2(-12, -12), Vec2(12, 12), 0.12);
        Real iso = 0.0;
        switch (cases[i].mode) {
            case 0:
                fieldApply(f, Op::Unite, sdfCircle(Vec2(-3, 0), 6.0));
                fieldApply(f, Op::Unite, sdfCircle(Vec2(3.6, 1.5), 5.0));
                break;
            case 1:
                fieldApply(f, Op::Unite, sdfCircle(Vec2(-1, 0), 7.0));
                fieldApply(f, Op::Subtract, sdfCircle(Vec2(4.5, 2.0), 5.0));
                break;
            case 2:
                fieldApply(f, Op::Unite, sdfCircle(Vec2(-2.5, 0), 6.5));
                fieldApply(f, Op::Intersect, sdfCircle(Vec2(2.5, 0), 6.5));
                break;
            default:
                fieldApply(f, Op::Unite, sdfBox(Rect(-7, -5, 2, 5)));
                fieldApply(f, Op::Unite, sdfCircle(Vec2(3.5, 0), 4.2));
                iso = 1.5;
                break;
        }
        if (cases[i].mode == 3) {
            for (const Poly2& l : fieldContour(f, 0.0))
                s.poly(l, "none", "#c8c2b4", 1.0, "stroke-dasharray=\"5 4\"");
        }
        int loops = 0;
        for (const Poly2& l : fieldContour(f, iso)) { s.poly(l, "#b9c0cc", "#232a36", 2.0); ++loops; }
        char buf[96];
        snprintf(buf, sizeof buf, "%s", cases[i].t);
        s.textPx(s.ox - 60, s.oy + 100, buf, 12.5, "#1d2430", "start", "600");
        snprintf(buf, sizeof buf, "%s", cases[i].d);
        s.textPx(s.ox - 60, s.oy + 116, buf, 10.5, "#9aa1ad");
    }
    (void)seed;
    svgClose(s);
}

}  // namespace lab

int main(int argc, char** argv) {
    const char* out = getenv("OUT") ? getenv("OUT") : ".";
    const uint32_t seed = argc > 1 ? static_cast<uint32_t>(atoi(argv[1])) : 7u;
    lab::sheetLots(out, seed);
    lab::sheetPlans(out, seed);
    lab::sheetShapes(out, seed);
    lab::sheetCurves(out, seed);
    printf("lot_lab: wrote %s/{lots,plans,shapes,curves}.svg (seed %u)\n", out, seed);
    return 0;
}
