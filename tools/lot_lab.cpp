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
// Build (from the repo root) — see tools/lot_lab_build.sh, which links the real
// city module so `sheetEngine` can drive the shipping pipeline headlessly:
//
//   ./tools/lot_lab_build.sh && OUT=/tmp /tmp/lot_lab 7
//
// Run:
//   /tmp/lot_lab              # writes lots.svg / plans.svg / shapes.svg to .
//   OUT=/tmp /tmp/lot_lab 12  # ...to /tmp, seed 12
//
// Pure, seeded and deterministic, like the procgen it prototypes: a sheet is
// reproducible from its seed.
// ---------------------------------------------------------------------------

#include "../src/engine/procgen/city/polygon.h"
// THE ENGINE ITSELF. `sheetEngine` drives the real road → block → parcel →
// architect → building pipeline headlessly and draws what it actually produced.
// Every other sheet on this tool is a PROTOTYPE of proposed ops that do not
// exist in the engine yet; this one is ground truth, and it is what the
// proposals get measured against.
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/parcel.h"

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
// Human-scale constants the grammar already pins (shape_grammar.h, ADR-0038 §5).
constexpr Real human_DOOR_WIDTH = 2.0;

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

// --- GENERAL polygon boolean (any angle, not just axis-aligned) ------------
//
// The rectilinear tracer above is exact and fast but only knows about boxes; a
// pentagon's walls are at 72 degrees and it has nothing to say about them. This
// is the general exact path, and it is the plan's claim made good: the engine's
// `polygonUnion` (road_offset.cpp, built to weld road junctions) already does
// arbitrary-angle UNION by splitting every edge at its crossings and keeping
// the sub-edges whose midpoint is outside the other polygons. Subtract and
// intersect are THE SAME ALGORITHM with a different keep rule — plus, for
// subtract, reversing the clip's surviving edges so the hole winds the other
// way. Reimplemented here so the lab can exercise all three.

enum class BOp { Unite, Subtract, Intersect };

// Even-odd containment across a whole loop set, so holes count correctly.
static bool inLoops(const std::vector<Poly2>& L, const Vec2& p) {
    int c = 0;
    for (const Poly2& l : L)
        if (l.size() >= 3 && engine::pointInPolygon(l, p)) c ^= 1;
    return c != 0;
}

static bool segCross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d, Real& t) {
    const Vec2 r = b - a, s = d - c;
    const Real den = engine::cross(r, s);
    if (std::fabs(den) < 1e-12) return false;
    const Real tt = engine::cross(c - a, s) / den;
    const Real uu = engine::cross(c - a, r) / den;
    if (tt < -1e-9 || tt > 1 + 1e-9 || uu < -1e-9 || uu > 1 + 1e-9) return false;
    t = std::max(Real(0), std::min(Real(1), tt));
    return true;
}

static std::vector<Poly2> polyBool(const std::vector<Poly2>& A,
                                   const std::vector<Poly2>& B, BOp op) {
    struct E { Vec2 a, b; int set; };
    std::vector<E> edges;
    auto gather = [&](const std::vector<Poly2>& S, int set) {
        for (const Poly2& p : S) {
            if (p.size() < 3) continue;
            for (std::size_t i = 0; i < p.size(); ++i)
                edges.push_back({p[i], p[(i + 1) % p.size()], set});
        }
    };
    gather(A, 0);
    gather(B, 1);
    if (edges.empty()) return {};

    std::vector<std::pair<Vec2, Vec2>> kept;
    for (const E& e : edges) {
        std::vector<Real> ts{0.0, 1.0};
        for (const E& o : edges) {
            if (o.set == e.set) continue;
            Real t;
            if (segCross(e.a, e.b, o.a, o.b, t)) ts.push_back(t);
        }
        std::sort(ts.begin(), ts.end());
        for (std::size_t k = 0; k + 1 < ts.size(); ++k) {
            if (ts[k + 1] - ts[k] < 1e-7) continue;
            const Vec2 P = e.a + (e.b - e.a) * ts[k];
            const Vec2 Q = e.a + (e.b - e.a) * ts[k + 1];
            const Vec2 mid = (P + Q) * 0.5;
            const bool inOther = inLoops(e.set == 0 ? B : A, mid);
            bool keep = false, flip = false;
            switch (op) {
                case BOp::Unite:                       // outside the other side
                    keep = !inOther; break;
                case BOp::Intersect:                   // inside the other side
                    keep = inOther; break;
                case BOp::Subtract:                    // A outside B; B inside A, reversed
                    keep = (e.set == 0) ? !inOther : inOther;
                    flip = (e.set == 1);
                    break;
            }
            if (keep) kept.push_back(flip ? std::make_pair(Q, P) : std::make_pair(P, Q));
        }
    }
    if (kept.empty()) return {};

    // Chain head-to-tail; at a shared vertex take the most-clockwise turn so the
    // walk hugs the boundary instead of cutting across it.
    const Real tol = 1e-4;
    auto key = [&](const Vec2& v) {
        return std::make_pair(static_cast<long long>(std::llround(v.x / tol)),
                              static_cast<long long>(std::llround(v.y / tol)));
    };
    std::map<std::pair<long long, long long>, std::vector<int>> byStart;
    for (int i = 0; i < static_cast<int>(kept.size()); ++i)
        byStart[key(kept[i].first)].push_back(i);
    std::vector<char> used(kept.size(), 0);
    std::vector<Poly2> loops;
    for (int s = 0; s < static_cast<int>(kept.size()); ++s) {
        if (used[s]) continue;
        Poly2 loop;
        int cur = s;
        while (cur != -1 && !used[cur]) {
            used[cur] = 1;
            loop.push_back(kept[cur].first);
            const Vec2 din = engine::normalize(kept[cur].second - kept[cur].first);
            int next = -1;
            Real best = -1e9;
            auto it = byStart.find(key(kept[cur].second));
            if (it != byStart.end()) {
                for (int cand : it->second) {
                    if (used[cand]) continue;
                    const Vec2 dout = engine::normalize(kept[cand].second - kept[cand].first);
                    const Real ang = std::atan2(engine::cross(din, dout), engine::dot(din, dout));
                    if (-ang > best) { best = -ang; next = cand; }
                }
            }
            cur = next;
        }
        if (loop.size() >= 3 && engine::area(loop) > 1e-3) loops.push_back(loop);
    }
    return loops;
}

// Fold a pile of polygons together with one op.
static std::vector<Poly2> polyBoolAll(std::vector<Poly2> acc,
                                      const std::vector<Poly2>& more, BOp op) {
    for (const Poly2& p : more) acc = polyBool(acc, {p}, op);
    return acc;
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

// Orient a set of contour loops by NESTING rather than by winding: a loop
// contained in an even number of others is an outer ring (forced CCW), an odd
// number makes it a hole (forced CW). Marching squares hands back whatever
// direction its cell walk happened to produce, so the winding sign it returns
// says nothing — this is what makes a field result interchangeable with an
// exact-boolean result downstream.
static std::vector<Poly2> orientByNesting(std::vector<Poly2> loops) {
    for (std::size_t i = 0; i < loops.size(); ++i) {
        if (loops[i].size() < 3) continue;
        int depth = 0;
        for (std::size_t j = 0; j < loops.size(); ++j) {
            if (i == j || loops[j].size() < 3) continue;
            if (engine::area(loops[j]) > engine::area(loops[i]) &&
                engine::pointInPolygon(loops[j], loops[i][0]))
                ++depth;
        }
        const bool wantCCW = (depth % 2) == 0;
        if ((engine::signedArea(loops[i]) > 0) != wantCCW)
            std::reverse(loops[i].begin(), loops[i].end());
    }
    return loops;
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

// ===========================================================================
// THE BLUEPRINT: walls with thickness, and OPENINGS decided on the plan.
//
// The move that makes the 2D layer load-bearing rather than decorative. Today
// `emitFacadeRect` computes its bays privately, in 3D, per wall rectangle — so
// nothing else can know where a window is, and nothing can be reviewed before
// geometry exists. Here the plan owns it:
//
//   an opening is a 1-D SPAN ALONG A WALL (the plan's business) plus a
//   SILL/HEAD height (the storey's business).
//
// Together those two facts fully determine the 3-D opening, so the blueprint
// becomes the authority and the 3-D pass stops deciding and starts reading.
// Everything below is drawable, reviewable, and testable with no renderer.
// ===========================================================================

enum class WallRole : uint8_t { Street, Side, Rear, Party, Court };

struct Opening {
    std::size_t edge = 0;
    Real s0 = 0, s1 = 0;      // span along the edge, metres from its start
    bool door = false;
    int swing = 1;            // which way the leaf opens, for the plan symbol
    Real sill = 0.9, head = 2.4;   // the storey's half of the fact (metres)
};

// ===========================================================================
// FENESTRATION: a small CLOSED set of strategies in code, an open set of
// configurations in data.
//
// The line this draws: **code owns verbs, data owns nouns and numbers.** A
// strategy knows HOW to glaze a wall — punch discrete openings on a module,
// run a horizontal ribbon, glaze the whole thing as a curtain, leave it blank.
// A recipe says WHICH strategy each wall gets and with what numbers. Adding
// "Edwardian" or "brutalist" must be data only; adding a genuinely new kind of
// geometry (oriels, a mansard) is one new strategy, and then every variant of
// it is data again.
//
// Test for the line being in the right place: **if a new architectural style
// requires a C++ change, the line is wrong.**
// ===========================================================================

enum class Fen : uint8_t {
    Punched,      // discrete openings on a bay module (traditional)
    Plate,        // one large fixed pane per wall (modern)
    Ribbon,       // a continuous horizontal band (moderne / offices)
    Curtain,      // the wall IS the glazing (towers)
    Clerestory,   // a high strip, solid below (industrial)
    Storefront,   // full-height glazing at grade (retail)
    Blank,        // nothing, by construction (party walls)
};

// One rule: which walls it matches, and how to glaze them. A real system's
// selector is richer (§6.2 — edge tag, floor range, bay index, corner
// convexity); role is enough to make the point.
struct WallRule {
    WallRole role = WallRole::Street;
    Fen fen = Fen::Punched;
    Real bay = 3.0;          // Punched: target module
    Real winW = 1.25;        // Punched: pane width (constant across walls)
    Real margin = 1.2;       // Plate/Ribbon/Clerestory: inset from each end
    Real sill = 0.9, head = 2.4;
    bool door = false;       // this wall carries the entrance
};

// A building recipe's fenestration table. THIS is the thing that would live in
// Lua; nothing below it changes when a new style is added.
struct FenRecipe {
    const char* name;
    const char* blurb;
    std::vector<WallRule> rules;   // matched by role; first match wins
    Real cornerWrap = 0;           // >0: corner glazing wrapping this far each way
    Real cornerSill = 0.4, cornerHead = 2.6;
};

// The WALL PROGRAM: partition one wall into bays and place its openings. This
// is the existing facade logic, lifted out of 3-D and made reviewable — same
// constant-module rule (a wide wall and a narrow one show the SAME window, the
// piers absorb the slack) that ADR-0040 already asks for.
static std::vector<Opening> wallProgram(std::size_t edge, Real length, WallRole role,
                                        Real bayW, int doorBay = -1) {
    std::vector<Opening> out;
    if (role == WallRole::Party || length < 1.2) return {};      // blank by construction
    const int bays = std::max(1, static_cast<int>(std::lround(length / bayW)));
    const Real bw = length / bays;
    const Real winW = std::min(Real(1.25), std::max(Real(0.7), bw - 0.9));
    for (int b = 0; b < bays; ++b) {
        const Real c = bw * (b + 0.5);
        if (role == WallRole::Street && b == doorBay) {
            const Real dw = std::min(human_DOOR_WIDTH, bw - 0.5);
            out.push_back({edge, c - dw * 0.5, c + dw * 0.5, true, (b % 2) ? 1 : -1, 0.0, 2.1});
        } else {
            const Real sill = (role == WallRole::Rear) ? 1.1 : 0.9;
            out.push_back({edge, c - winW * 0.5, c + winW * 0.5, false, 1, sill, 2.4});
        }
    }
    return out;
}

// Glaze one wall run [a,b] (metres along the edge) with one strategy. This is
// the ONLY place geometry decisions live — and it is a closed switch.
static void glazeRun(std::vector<Opening>& out, std::size_t edge, Real a, Real b,
                     const WallRule& r) {
    const Real len = b - a;
    if (len < 0.8) return;
    switch (r.fen) {
        case Fen::Blank:
            return;
        case Fen::Curtain: {                       // the wall IS the opening
            out.push_back({edge, a + 0.12, b - 0.12, false, 1, 0.15, 3.0});
            return;
        }
        case Fen::Storefront: {
            if (len < 2.0) return;
            out.push_back({edge, a + 0.5, b - 0.5, false, 1, 0.25, 2.9});
            return;
        }
        case Fen::Plate: {
            const Real m = std::min(r.margin, len * 0.28);
            if (len - 2 * m < 1.0) return;
            out.push_back({edge, a + m, b - m, false, 1, r.sill, r.head});
            return;
        }
        case Fen::Ribbon:
        case Fen::Clerestory: {
            const Real m = std::min(r.margin, len * 0.18);
            if (len - 2 * m < 1.0) return;
            out.push_back({edge, a + m, b - m, false, 1, r.sill, r.head});
            return;
        }
        case Fen::Punched:
        default: {
            const int bays = std::max(1, static_cast<int>(std::lround(len / r.bay)));
            const Real bw = len / bays;
            // Constant module: a wide wall and a narrow one show the SAME pane;
            // the piers take the slack (ADR-0040).
            const Real w = std::min(r.winW, std::max(Real(0.6), bw - 0.9));
            for (int i = 0; i < bays; ++i) {
                const Real c = a + bw * (i + 0.5);
                out.push_back({edge, c - w * 0.5, c + w * 0.5, false, 1,
                               r.sill, r.head});
            }
            return;
        }
    }
}

// FENESTRATE a whole plan. This runs per PLAN, not per wall — because a corner
// window spans TWO walls and deletes the post between them, which a per-wall
// function structurally cannot express. Corners are claimed first, then each
// wall's remaining run is glazed by its rule.
static std::vector<Opening> fenestrate(const Poly2& plan,
                                       const std::vector<WallRole>& roles,
                                       const FenRecipe& rec) {
    const std::size_t n = plan.size();
    std::vector<Opening> out;
    std::vector<Real> claimStart(n, 0.0), claimEnd(n, 0.0);   // metres claimed at each end

    auto ruleFor = [&](WallRole role) -> const WallRule* {
        for (const WallRule& r : rec.rules) if (r.role == role) return &r;
        return rec.rules.empty() ? nullptr : &rec.rules.front();
    };

    // 1. Corner glazing claims a span off BOTH walls meeting at a convex corner.
    if (rec.cornerWrap > 0) {
        for (std::size_t v = 0; v < n; ++v) {
            const std::size_t prev = (v + n - 1) % n;
            const Vec2 d0 = engine::normalize(plan[v] - plan[prev]);
            const Vec2 d1 = engine::normalize(plan[(v + 1) % n] - plan[v]);
            if (engine::cross(d0, d1) <= 1e-6) continue;   // CCW: convex is cross > 0
            const Real lPrev = (plan[v] - plan[prev]).length();
            const Real lNext = (plan[(v + 1) % n] - plan[v]).length();
            const Real w = std::min(rec.cornerWrap,
                                    std::min(lPrev, lNext) * 0.42);
            if (w < 0.5) continue;
            if (roles[prev] == WallRole::Party || roles[v] == WallRole::Party) continue;
            out.push_back({prev, lPrev - w, lPrev, false, 1, rec.cornerSill, rec.cornerHead});
            out.push_back({v, 0.0, w, false, 1, rec.cornerSill, rec.cornerHead});
            claimEnd[prev] = std::max(claimEnd[prev], w);
            claimStart[v] = std::max(claimStart[v], w);
        }
    }
    // 2. Each wall glazes what is left of it, by its own rule.
    for (std::size_t i = 0; i < n; ++i) {
        const WallRule* r = ruleFor(roles[i]);
        if (!r) continue;
        const Real len = (plan[(i + 1) % n] - plan[i]).length();
        const Real a = claimStart[i] + (claimStart[i] > 0 ? 0.35 : 0.0);
        const Real b = len - claimEnd[i] - (claimEnd[i] > 0 ? 0.35 : 0.0);
        // The DOOR is orthogonal to the glazing strategy: a plate-glass wall
        // still needs an entrance. Place it first, then glaze what remains on
        // either side — so every strategy gets a door without knowing about it.
        if (r->door && b - a > human_DOOR_WIDTH + 1.2) {
            const Real c = (a + b) * 0.5, dw = human_DOOR_WIDTH;
            out.push_back({i, c - dw * 0.5, c + dw * 0.5, true, -1, 0.0, 2.1});
            glazeRun(out, i, a, c - dw * 0.5 - 0.35, *r);
            glazeRun(out, i, c + dw * 0.5 + 0.35, b, *r);
        } else {
            glazeRun(out, i, a, b, *r);
        }
    }
    return out;
}

// The opening's footprint through the wall, for cutting the poche.
static Poly2 openingRect(const Poly2& p, const Opening& o, Real t) {
    const std::size_t n = p.size();
    const Vec2 a = p[o.edge % n], b = p[(o.edge + 1) % n];
    const Vec2 u = engine::normalize(b - a);
    const Vec2 out(u.y, -u.x);                    // outward: right of a CCW edge
    Poly2 r{a + u * o.s0 + out * 0.3, a + u * o.s1 + out * 0.3,
            a + u * o.s1 - out * (t + 0.3), a + u * o.s0 - out * (t + 0.3)};
    if (engine::signedArea(r) < 0) std::reverse(r.begin(), r.end());
    return r;
}

// The drawn wall: an outer ring, an inner ring `t` in from it, and every
// opening punched clean through both. Three booleans, no special cases.
static std::vector<Poly2> wallBand(const Poly2& outer, Real t,
                                   const std::vector<Opening>& ops) {
    const Poly2 inner = insetEdges(outer, std::vector<Real>(outer.size(), t));
    if (inner.size() < 3) return {outer};
    std::vector<Poly2> band = polyBool({outer}, {inner}, BOp::Subtract);
    for (const Opening& o : ops)
        band = polyBool(band, {openingRect(outer, o, t)}, BOp::Subtract);
    return band;
}

// ---------------------------------------------------------------------------
// Sheet 5 — THE COMPOSITION TEST. Do the pieces actually work together? Start
// from a pentagon, grow a wing off every wall, cap the wings with drums, weld
// the joints with a smooth-min field, and subtract a court from the middle.
// Exact loop algebra and the sampled field, on one shape, in one pipeline.
// ---------------------------------------------------------------------------

// Exact signed distance to a simple polygon: nearest point on the boundary,
// signed by containment. Works for any simple polygon, convex or not.
static Real sdfPolyAt(const Poly2& p, const Vec2& q) {
    Real best = 1e30;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2& a = p[i];
        const Vec2& b = p[(i + 1) % p.size()];
        const Vec2 ab = b - a;
        const Real l2 = ab.lengthSquared();
        Real t = l2 > 1e-12 ? engine::dot(q - a, ab) / l2 : 0;
        t = std::max(Real(0), std::min(Real(1), t));
        best = std::min(best, (q - (a + ab * t)).length());
    }
    return engine::pointInPolygon(p, q) ? -best : best;
}
// Polynomial smooth minimum — the WELD. k is the blend radius in metres, so a
// joint gets a real fillet instead of a crease.
static Real smoothMin(Real a, Real b, Real k) {
    if (k <= 1e-6) return std::min(a, b);
    const Real h = std::max(Real(0), std::min(Real(1), 0.5 + 0.5 * (b - a) / k));
    return b * (1 - h) + a * h - k * h * (1 - h);
}

static Poly2 regularPoly(const Vec2& c, Real r, int n, Real rot) {
    Poly2 p;
    for (int i = 0; i < n; ++i) {
        const Real t = rot + kTau * i / n;
        p.push_back(Vec2(c.x + r * std::cos(t), c.y + r * std::sin(t)));
    }
    return p;
}

static void sheetCompose(const char* dir, uint32_t seed) {
    // --- the pieces -------------------------------------------------------
    const Vec2 O(0, 0);
    const Real R = 13.0;
    const Poly2 pent = regularPoly(O, R, 5, -kTau * 0.25);

    std::vector<Poly2> wings, drums;
    for (std::size_t i = 0; i < pent.size(); ++i) {
        const Vec2 a = pent[i], b = pent[(i + 1) % pent.size()];
        const Vec2 u = engine::normalize(b - a);
        const Vec2 n(u.y, -u.x);                 // outward: right of a CCW edge
        const Vec2 m = (a + b) * 0.5;
        const Real hw = 3.1, depth = 7.5;
        Poly2 w{m - u * hw - n * 1.5, m + u * hw - n * 1.5,
                m + u * hw + n * depth, m - u * hw + n * depth};
        if (engine::signedArea(w) < 0) std::reverse(w.begin(), w.end());
        wings.push_back(w);
        drums.push_back(regularPoly(m + n * depth, 4.2, 28, 0));
    }
    const Poly2 court = regularPoly(O, R * 0.46, 5, -kTau * 0.25 + kTau * 0.1);

    // --- exact path -------------------------------------------------------
    std::vector<Poly2> step1{pent};
    std::vector<Poly2> step2 = polyBoolAll(step1, wings, BOp::Unite);
    std::vector<Poly2> step3 = polyBoolAll(step2, drums, BOp::Unite);
    std::vector<Poly2> step5 = polyBool(step3, {court}, BOp::Subtract);

    // --- field path: the same pieces, welded ------------------------------
    auto buildField = [&](Real k, bool cut) {
        Field f = fieldMake(Vec2(-30, -30), Vec2(30, 30), 0.14);
        std::vector<const Poly2*> all{&pent};
        for (const Poly2& w : wings) all.push_back(&w);
        for (const Poly2& d : drums) all.push_back(&d);
        for (int j = 0; j < f.ny; ++j)
            for (int i = 0; i < f.nx; ++i) {
                const Vec2 q = f.pos(i, j);
                Real d = 1e30;
                for (const Poly2* s : all) d = smoothMin(d, sdfPolyAt(*s, q), k);
                if (cut) d = std::max(d, -sdfPolyAt(court, q));   // SDF subtract
                f.d[static_cast<std::size_t>(j) * f.nx + i] = d;
            }
        return f;
    };

    struct Panel {
        const char* t;
        const char* d;
        std::vector<Poly2> loops;
        bool field = false;
    };
    std::vector<Panel> panels;
    panels.push_back({"1 · seed", "a pentagon — the starting shape", step1, false});
    panels.push_back({"2 · wings", "one wing pushed off every wall (exact union)", step2, false});
    panels.push_back({"3 · drums", "a circle joined to each wing end (exact union)", step3, false});
    panels.push_back({"4 · welded", "same pieces, smooth-min field: joints fillet",
                      orientByNesting(fieldContour(buildField(2.6, false))), true});
    panels.push_back({"5 · court cut", "inner pentagon subtracted (exact boolean)", step5, false});
    panels.push_back({"6 · composite", "welded mass minus the court — one pipeline",
                      orientByNesting(fieldContour(buildField(2.6, true))), true});

    const Real cellW = 400, cellH = 400;
    const int cols = 3, rows = 2;
    Svg s;
    svgOpen(s, (std::string(dir) + "/compose.svg").c_str(), cellW * cols + 60,
            cellH * rows + 170, "Lot Lab — do the pieces compose?");
    s.textPx(28, 68, "Pentagon -> a wing off every wall -> drums on the wing ends -> smooth-min "
                     "weld -> subtract a court. Steps 1/2/3/5 are the EXACT arbitrary-angle "
                     "boolean; 4 and 6 are the sampled field. Same shapes, one pipeline.",
             12, "#6b7280");

    const Real drawW = cellW - 90, drawH = 250;
    for (std::size_t i = 0; i < panels.size(); ++i) {
        const int cx = static_cast<int>(i) % cols, cy = static_cast<int>(i) / cols;
        const Real bx = 40 + cx * cellW + 30, by = 118 + cy * cellH;
        s.scale = std::min(drawW / 62.0, drawH / 62.0);
        s.ox = bx + drawW * 0.5;
        s.oy = by + drawH * 0.5;

        // Ghost the input pieces behind the result, so each step SHOWS its move.
        if (i >= 1 && i <= 3) {
            for (std::size_t k = 0; k < wings.size(); ++k) {
                if (i >= 2) s.poly(wings[k], "none", "#c9c3b4", 0.8, "stroke-dasharray=\"3 3\"");
                if (i >= 3) s.poly(drums[k], "none", "#c9c3b4", 0.8, "stroke-dasharray=\"3 3\"");
            }
        }
        if (i == 4 || i == 5) s.poly(court, "none", "#d3a08e", 1.0, "stroke-dasharray=\"4 3\"");

        Real ar = 0;
        int holes = 0;
        for (const Poly2& l : panels[i].loops) {
            const bool hole = engine::signedArea(l) < 0;
            if (hole) ++holes;
            s.poly(l, hole ? "#faf8f3" : "#b9c0cc", "#232a36", 2.0);
            ar += hole ? -engine::area(l) : engine::area(l);
        }
        char buf[192];
        snprintf(buf, sizeof buf, "%s", panels[i].t);
        s.textPx(bx, by + drawH + 44, buf, 14, "#1d2430", "start", "600");
        snprintf(buf, sizeof buf, "%s", panels[i].d);
        s.textPx(bx, by + drawH + 61, buf, 11, "#6b7280");
        snprintf(buf, sizeof buf, "%.0f m2  ·  %d loop(s), %d hole(s)  ·  %s",
                 ar, static_cast<int>(panels[i].loops.size()), holes,
                 panels[i].field ? "sampled field" : "exact boolean");
        s.textPx(bx, by + drawH + 76, buf, 10, "#9aa1ad");
    }
    (void)seed;
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 6 — BLUEPRINTS. The same plans, drawn as construction documents:
// walls with real thickness, doors with swings, windows with glazing lines.
// ---------------------------------------------------------------------------
static void drawBlueprint(Svg& s, const Poly2& plan, Real t,
                          const std::vector<Opening>& ops) {
    // Floor area first, so the poche reads as walls sitting on a slab.
    s.poly(plan, "#eceadf", "none", 0);
    for (const Poly2& b : wallBand(plan, t, ops))
        s.poly(b, "#3a4252", "#232a36", 0.7);

    const std::size_t n = plan.size();
    for (const Opening& o : ops) {
        const Vec2 a = plan[o.edge % n], b = plan[(o.edge + 1) % n];
        const Vec2 u = engine::normalize(b - a);
        const Vec2 inw(-u.y, u.x);                   // into the room
        const Vec2 P0 = a + u * o.s0, P1 = a + u * o.s1;
        if (o.door) {
            // Standard plan symbol: leaf swung 90 deg off the hinge, plus the
            // arc it sweeps. This is the cutaway that reads as a door.
            const Vec2 hinge = o.swing > 0 ? P0 : P1;
            const Vec2 latch = o.swing > 0 ? P1 : P0;
            const Real w = (P1 - P0).length();
            const Vec2 leaf = hinge + inw * w;
            s.line(hinge - inw * (t * 0.5), leaf, "#1d2430", 1.6);
            s.arc(hinge, latch, o.swing > 0 ? 90.0 : -90.0, "#8e97a6", 0.9);
        } else {
            // Glazing: a pair of thin lines set in the reveal, plus jambs.
            s.line(P0 - inw * (t * 0.35), P1 - inw * (t * 0.35), "#4a7fa8", 1.0);
            s.line(P0 - inw * (t * 0.65), P1 - inw * (t * 0.65), "#4a7fa8", 1.0);
            s.line(P0, P0 - inw * t, "#232a36", 0.7);
            s.line(P1, P1 - inw * t, "#232a36", 0.7);
        }
    }
}

// ===========================================================================
// LOT CONTEXT — where "is this a corner?" actually comes from.
//
// Cornerness is NOT a property of the building, and it is not derivable from
// the plan. It is a fact about the PARCEL'S PLACE IN THE BLOCK: what lies on
// the far side of each of its edges. The parcelling pass knows that (it just
// cut the block), so it tags every parcel edge as it emits it, and everything
// downstream — the plan grammar, the fenestration table, the architect's
// recipe pick — selects on the tag instead of re-deriving it.
//
// This is the piece that makes `EdgeTag` more than a label: the tag has a
// SOURCE, and the source is the only place that can know.
// ===========================================================================

enum class LotShape : uint8_t {
    MidBlock,    // one street frontage
    Corner,      // two ADJACENT street frontages — the interesting case
    Through,     // two OPPOSITE frontages (street front, lane behind)
    Island,      // street on every side
};

struct LotContext {
    LotShape shape = LotShape::MidBlock;
    std::vector<WallRole> roles;    // per edge, from the block topology
    int cornerVertex = -1;          // the vertex where two street edges meet
    Real cornerAngle = 0;           // interior angle there (radians)
    int frontages = 0;
};

// Tag each parcel edge by what is across it, then classify the parcel. `onBlock`
// answers "does this point sit on the block's street boundary", `onCore`
// "…on the block's interior core" — both trivially known to the parceller.
static LotContext classifyLot(const Poly2& lot,
                              const std::function<bool(const Vec2&)>& onBlock,
                              const std::function<bool(const Vec2&)>& onCore) {
    LotContext c;
    const std::size_t n = lot.size();
    c.roles.assign(n, WallRole::Party);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 m = (lot[i] + lot[(i + 1) % n]) * 0.5;
        if (onBlock(m))      { c.roles[i] = WallRole::Street; ++c.frontages; }
        else if (onCore(m))  { c.roles[i] = WallRole::Rear; }
        else                 { c.roles[i] = WallRole::Party; }
    }
    // Two ADJACENT street edges meet at a corner; two opposite ones make a
    // through lot. The distinction matters: only the first can be chamfered.
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        if (c.roles[i] == WallRole::Street && c.roles[j] == WallRole::Street) {
            c.shape = LotShape::Corner;
            c.cornerVertex = static_cast<int>(j);
            const Vec2 d0 = engine::normalize(lot[j] - lot[i]);
            const Vec2 d1 = engine::normalize(lot[(j + 1) % n] - lot[j]);
            c.cornerAngle = kTau * 0.5 - std::atan2(engine::cross(d0, d1),
                                                    engine::dot(d0, d1));
            break;
        }
    }
    if (c.shape != LotShape::Corner) {
        if (c.frontages >= static_cast<int>(n)) c.shape = LotShape::Island;
        else if (c.frontages >= 2)              c.shape = LotShape::Through;
    }
    return c;
}

// CHAMFER the corner: replace the vertex with a cut edge, and TAG that new edge
// street — it faces the junction. This is why the doorway can land on it: the
// plan grammar produces the edge, the tag makes it addressable, and the
// fenestration table puts the entrance there. Three layers, no special case.
static Poly2 chamferCorner(const Poly2& in, std::vector<WallRole>& roles,
                           std::size_t vtx, Real cut) {
    const std::size_t n = in.size();
    if (n < 3) return in;
    const Vec2 P = in[(vtx + n - 1) % n], V = in[vtx], N = in[(vtx + 1) % n];
    const Vec2 d0 = engine::normalize(V - P), d1 = engine::normalize(N - V);
    const Real c = std::min(cut, std::min((V - P).length(), (N - V).length()) * 0.45);
    Poly2 out;
    std::vector<WallRole> r;
    for (std::size_t i = 0; i < n; ++i) {
        if (i == vtx) {
            out.push_back(V - d0 * c);
            r.push_back(WallRole::Street);      // the CUT edge faces the junction
            out.push_back(V + d1 * c);
            r.push_back(roles[i]);
        } else {
            out.push_back(in[i]);
            r.push_back(roles[i]);
        }
    }
    roles = r;
    return out;
}

// ===========================================================================
// THE STACK SPEC — where floor heights and the loft come from.
//
// A building is a list of STOREY BANDS. Height belongs to the band, not to the
// building: that is what lets one tower carry a 5.2 m lobby, forty 3.9 m office
// floors, a 4.5 m plant level and a 5 m crown. The current `floorHeight` +
// `groundHeight` pair can express exactly two of those.
// ===========================================================================

enum class BandRole : uint8_t { Lobby, Retail, Office, Residential, Plant, Crown, Parking };

struct StoreyBand {
    BandRole role;
    int storeys;
    Real height;              // floor-to-floor for THIS band
    const char* plan;         // which plan-grammar script this band runs on
    const char* loftTo;       // nullptr = prismatic; else morph toward that plan
    Real twist;               // total twist across the band (radians)
};

static const char* bandName(BandRole r) {
    switch (r) {
        case BandRole::Lobby:       return "lobby";
        case BandRole::Retail:      return "retail";
        case BandRole::Office:      return "office";
        case BandRole::Residential: return "residential";
        case BandRole::Plant:       return "plant";
        case BandRole::Parking:     return "parking";
        default:                    return "crown";
    }
}

static std::vector<Poly2> loftPlans(const std::vector<Poly2>& A,
                                    const std::vector<Poly2>& B, Real t,
                                    Real twistRad, Real cell);
static Poly2 regularPoly(const Vec2& c, Real r, int n, Real rot);

// ===========================================================================
// THE 2-D PEN — the turtle from §3.4, built. A plan is walked, not typed as
// vertex literals, which is what makes irregular plans authorable as data.
// ===========================================================================
struct Pen {
    Poly2 pts;
    Vec2 cur{0, 0};
    Real dir = 0;                       // heading, radians

    Pen& moveTo(Real x, Real y) { cur = Vec2(x, y); pts.push_back(cur); return *this; }
    Pen& turn(Real deg) { dir += deg * kTau / 360.0; return *this; }
    Pen& forward(Real d) {
        cur = cur + Vec2(std::cos(dir), std::sin(dir)) * d;
        pts.push_back(cur);
        return *this;
    }
    // Sweep an arc of `deg` while advancing — a curved wall, as chords.
    Pen& arc(Real radius, Real deg, int segs = 8) {
        const Real step = deg / segs;
        const Real chord = 2.0 * radius * std::sin(std::fabs(step) * kTau / 720.0);
        for (int i = 0; i < segs; ++i) { turn(step * 0.5); forward(chord); turn(step * 0.5); }
        return *this;
    }
    Poly2 close() {
        Poly2 p = pts;
        if (p.size() > 2 && (p.front() - p.back()).length() < 1e-6) p.pop_back();
        if (engine::signedArea(p) < 0) std::reverse(p.begin(), p.end());
        return p;
    }
};

// ===========================================================================
// VERTICAL DESCRIPTION — three INDEPENDENT knobs, not one.
//
//   plan     WHAT the footprint is        (a plan-grammar script)
//   profile  HOW BIG it is at height t    (a scalar curve — the Gherkin)
//   loft     WHAT IT BECOMES              (morph toward another plan — the arch)
//   twist    HOW IT ROTATES               (rarely; see the quota note below)
//
// A Gherkin is ONE plan with a profile curve and no loft at all. A pyramid is
// one plan with a linear profile. An arch is a loft that MERGES two loops into
// one going up. Conflating "profile" with "loft" is why a system ends up
// twisting everything: twist is the only knob it has for "make it interesting".
// ===========================================================================

using Profile = std::function<Real(Real)>;      // t in [0,1] -> scale

static Profile profileConstant() { return [](Real) { return Real(1); }; }
static Profile profileTaper(Real to) {          // pyramid / obelisk
    return [to](Real t) { return 1.0 + (to - 1.0) * t; };
}
static Profile profileSwell() {                 // the Gherkin: bulge, then nose
    return [](Real t) {
        return std::pow(std::sin(kTau * 0.5 * (0.20 + 0.78 * t)), Real(0.85));
    };
}

// One storey's plan: the band's plan, lofted if asked, then scaled by the
// profile, then twisted. Order matters — profile is a size, twist is a frame.
static std::vector<Poly2> storeyPlan(const std::vector<Poly2>& A,
                                     const std::vector<Poly2>* B, Real t,
                                     const Profile& prof, Real twist) {
    std::vector<Poly2> loops = B ? loftPlans(A, *B, t, 0.0, 0.18) : A;
    const Real s = std::max(Real(0.02), prof(t));
    Vec2 c(0, 0);
    Real n = 0;
    for (const Poly2& p : loops) for (const Vec2& v : p) { c = c + v; n += 1; }
    if (n > 0) c = c * (1.0 / n);
    const Real cs = std::cos(twist), sn = std::sin(twist);
    for (Poly2& p : loops)
        for (Vec2& v : p) {
            Vec2 r = (v - c) * s;
            v = Vec2(c.x + r.x * cs - r.y * sn, c.y + r.x * sn + r.y * cs);
        }
    return loops;
}

// A centre SECTION built from the storey plans: for each level, the x-intervals
// the plans actually occupy along the building's mid-line. This is what makes an
// arch read as an arch and a donut read as a donut — the massing is legible from
// the plans alone, with no 3-D anywhere.
static void drawSection(Svg& s, const std::vector<std::vector<Poly2>>& levels,
                        Real x0, Real x1, Real yMid, Real px, Real pyBase,
                        Real sc, Real levelH, const char* fill) {
    const int NS = 150;
    for (std::size_t L = 0; L < levels.size(); ++L) {
        const Real yTop = pyBase - static_cast<Real>(L + 1) * levelH;
        int runStart = -1;
        for (int i = 0; i <= NS; ++i) {
            const Real x = x0 + (x1 - x0) * i / NS;
            bool inside = false;
            if (i < NS) {
                int c = 0;
                for (const Poly2& p : levels[L])
                    if (engine::pointInPolygon(p, Vec2(x, yMid))) c ^= 1;
                inside = c != 0;
            }
            if (inside && runStart < 0) runStart = i;
            if (!inside && runStart >= 0) {
                const Real a = px + (x0 + (x1 - x0) * runStart / NS - x0) * sc;
                const Real b = px + (x0 + (x1 - x0) * i / NS - x0) * sc;
                fprintf(s.f, "<rect x=\"%.2f\" y=\"%.2f\" width=\"%.2f\" height=\"%.2f\" "
                             "fill=\"%s\" stroke=\"#232a36\" stroke-width=\"0.4\"/>\n",
                        a, yTop, std::max(b - a, Real(0.8)), levelH + 0.4, fill);
                runStart = -1;
            }
        }
    }
}

// ===========================================================================
// BUILDABILITY — the inversion the plan was missing.
//
// Today: cut lots on a uniform rhythm, then REJECT the buildings that don't fit
// (six rejection counters in growLotBuildings). That is why a skinny trapezoid
// gets a tiny triangular house: nothing upstream ever asked whether a building
// could stand there.
//
// Correct: a program declares the smallest RECTANGLE it can be built on, and
// the parceller only emits lots that contain one. Land that can carry no
// program becomes open space BY DESIGN, not by rejection.
// ===========================================================================

// The largest rectangle (in the lot's own oriented frame) that fits inside it.
// Shrink-to-fit about the centroid — the same construction `RectYard` already
// uses to seat a house, promoted to a lot-qualifying measurement.
static void maxInscribedRect(const Poly2& lot, Real& outW, Real& outD) {
    outW = outD = 0;
    if (lot.size() < 3) return;
    const engine::OBB2 ob = engine::orientedBoundingBox(lot);
    Vec2 c = engine::centroid(lot);
    if (!engine::pointInPolygon(lot, c)) c = ob.center;
    Real hw = ob.half[0], hd = ob.half[1];
    for (int iter = 0; iter < 40; ++iter) {
        const Poly2 r{c - ob.axis[0] * hw - ob.axis[1] * hd,
                      c + ob.axis[0] * hw - ob.axis[1] * hd,
                      c + ob.axis[0] * hw + ob.axis[1] * hd,
                      c - ob.axis[0] * hw + ob.axis[1] * hd};
        bool ok = true;
        for (const Vec2& v : r) if (!engine::pointInPolygon(lot, v)) { ok = false; break; }
        if (ok) break;
        hw *= 0.93;
        hd *= 0.93;
    }
    outW = 2 * hw;
    outD = 2 * hd;
}

// What a program needs to exist at all. A lot that cannot hold this is not a
// lot for this program — full stop, decided before anything is grown.
struct Buildable {
    const char* program;
    Real minW, minD;      // the smallest rectangle the building needs
    Real minArea;
    int  maxStoreys;      // what this plate can structurally carry
};

// Minimum PLATE grows with height: lifts, cores and structure scale with the
// storeys they serve. This is the rule that stops a 100-storey tower landing on
// a 200 m² plot — expressed once, not as a per-recipe guess.
static int storeyCapFor(Real plateW, Real plateD) {
    const Real shortSide = std::min(plateW, plateD);
    const Real area = plateW * plateD;
    // ~1 storey per 0.55 m of short side, and an area floor per 12 storeys.
    const int bySide = static_cast<int>(shortSide / 0.55);
    const int byArea = static_cast<int>(area / 26.0);
    return std::max(1, std::min(bySide, byArea));
}

// PLAN QUALITY GATE — a real invariant, not a post-hoc rejection. A defect in
// the base plan is AMPLIFIED up the whole stack (the divot that rides a
// skyscraper's shaft for sixty storeys started as one bad vertex), so this is
// the highest-leverage check in the system and it runs before a plan is
// accepted, never after the geometry exists.
struct PlanDefect { bool ok = true; const char* why = ""; Real value = 0; };

static PlanDefect planQuality(const Poly2& p, Real minEdge = 2.2,
                              Real minAngleDeg = 42.0) {
    PlanDefect d;
    const std::size_t n = p.size();
    if (n < 3) { d.ok = false; d.why = "degenerate"; return d; }
    for (std::size_t i = 0; i < n; ++i) {
        const Real len = (p[(i + 1) % n] - p[i]).length();
        if (len < minEdge) { d.ok = false; d.why = "wall shorter than minimum"; d.value = len; return d; }
    }
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = p[(i + n - 1) % n], b = p[i], c = p[(i + 1) % n];
        const Vec2 d0 = engine::normalize(b - a), d1 = engine::normalize(c - b);
        const Real turn = std::atan2(engine::cross(d0, d1), engine::dot(d0, d1));
        const Real interior = 180.0 - std::fabs(turn) * 360.0 / kTau;
        if (interior < minAngleDeg) {
            d.ok = false; d.why = "corner too acute"; d.value = interior; return d;
        }
    }
    return d;
}

// ===========================================================================
// Sheet 0 — THE ENGINE, drawn. No prototype code below this line: every shape
// on this sheet came out of the shipping pipeline.
//
//   gridRoads -> planarize -> extractBlocks -> growLotBuildings
//
// This is the sheet that makes the tool honest. It is the baseline any proposal
// has to beat, and it is the diagnostic for changing the engine: alter
// `city_lots.cpp` or `architect.cpp`, rebuild, and see the difference on paper.
// ===========================================================================
static void sheetEngine(const char* dir, uint32_t seed) {
    engine::GridRoadParams gp;
    gp.extent = 150;
    gp.seed = seed;
    const engine::RoadGraph g = engine::planarize(engine::gridRoads(gp));
    const std::vector<Poly2> blocks = engine::extractBlocks(g, 200);

    engine::LotParams lp;
    lp.seed = seed;
    engine::LotPlanDebug dbg;
    const std::vector<engine::LotBuilding> built =
        engine::growLotBuildings(blocks, lp, &dbg);

    Svg s;
    const Real cellW = 470, cellH = 470;
    svgOpen(s, (std::string(dir) + "/engine.svg").c_str(), cellW * 3 + 60, cellH * 2 + 190,
            "Lot Lab — the ENGINE's own output (nothing on this sheet is a prototype)");
    s.textPx(28, 68, "gridRoads -> planarize -> extractBlocks -> growLotBuildings, run headlessly. "
                     "This is the baseline; change city_lots.cpp or architect.cpp, rebuild, and "
                     "the difference shows up here.", 12, "#6b7280");

    Vec2 lo, hi;
    engine::bounds(blocks.empty() ? Poly2{Vec2(-150, -150), Vec2(150, 150)} : blocks[0], lo, hi);
    for (const Poly2& b : blocks) {
        Vec2 a, c;
        engine::bounds(b, a, c);
        lo = Vec2(std::min(lo.x, a.x), std::min(lo.y, a.y));
        hi = Vec2(std::max(hi.x, c.x), std::max(hi.y, c.y));
    }
    const Real span = std::max(hi.x - lo.x, hi.y - lo.y) + 8;
    const Vec2 mid((lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5);

    auto frame = [&](int i, Real sp) {
        const int cx = i % 3, cy = i / 3;
        const Real bx = 40 + cx * cellW + 30, by = 118 + cy * cellH;
        s.scale = 330.0 / sp;
        s.ox = bx + 165 - mid.x * s.scale;
        s.oy = by + 165 - mid.y * s.scale;
        return std::make_pair(bx, by);
    };
    auto label = [&](Real bx, Real by, const char* t, const char* d, const char* st) {
        s.textPx(bx, by + 372, t, 14, "#1d2430", "start", "600");
        s.textPx(bx, by + 389, d, 10.5, "#6b7280");
        s.textPx(bx, by + 404, st, 10, "#9aa1ad");
    };
    char buf[220];

    // --- 1: the real road graph + the faces it encloses --------------------
    {
        auto [bx, by] = frame(0, span);
        for (const Poly2& b : blocks) s.poly(b, "#eceadf", "#c9c3b4", 1.0);
        for (const engine::RoadEdge& e : g.edges) {
            if (e.a < 0 || e.b < 0) continue;
            s.line(g.nodes[e.a].pos, g.nodes[e.b].pos, "#8a8578",
                   std::max(Real(0.7), e.width * s.scale * 0.10));
        }
        snprintf(buf, sizeof buf, "%zu nodes, %zu edges -> %zu blocks",
                 g.nodes.size(), g.edges.size(), blocks.size());
        label(bx, by, "1 · engine roads and blocks",
              "gridRoads -> planarize -> extractBlocks", buf);
    }
    // --- 2: the real parcels ------------------------------------------------
    {
        auto [bx, by] = frame(1, span);
        for (const Poly2& b : dbg.blocks) s.poly(b, "#f2eee5", "#c9c3b4", 0.8);
        for (const Poly2& l : dbg.lots) s.poly(l, "#dfe7d5", "#7d9a6c", 0.9);
        snprintf(buf, sizeof buf, "%zu block interiors -> %zu parcels",
                 dbg.blocks.size(), dbg.lots.size());
        label(bx, by, "2 · engine parcels",
              "subdivideBlock, inset by roadMargin first", buf);
    }
    // --- 3: what the engine actually built ----------------------------------
    {
        auto [bx, by] = frame(2, span);
        for (const Poly2& b : dbg.blocks) s.poly(b, "#f7f4ec", "#ddd8cc", 0.6);
        int greens = 0, parks = 0, bld = 0;
        for (const engine::LotBuilding& lb : built) {
            if (lb.type == "green") { ++greens; if (lb.pad.size() >= 3) s.poly(lb.pad, "#e2ecd8", "none", 0); continue; }
            if (lb.type == "park")  { ++parks;  if (lb.pad.size() >= 3) s.poly(lb.pad, "#cfe1c2", "#7d9a6c", 0.8); continue; }
            if (lb.plan.size() >= 3) { s.poly(lb.plan, "#b9c0cc", "#232a36", 1.0); ++bld; }
        }
        snprintf(buf, sizeof buf, "%d buildings, %d parks, %d greens", bld, parks, greens);
        label(bx, by, "3 · engine buildings",
              "growLotBuildings — these are its real plan polygons", buf);
    }
    // --- 4: one block, up close ---------------------------------------------
    {
        // Pick the largest block so the parcel grain is legible.
        std::size_t big = 0;
        Real bestA = 0;
        for (std::size_t i = 0; i < blocks.size(); ++i)
            if (engine::area(blocks[i]) > bestA) { bestA = engine::area(blocks[i]); big = i; }
        Vec2 blo, bhi;
        engine::bounds(blocks.empty() ? Poly2{} : blocks[big], blo, bhi);
        const Vec2 bmid((blo.x + bhi.x) * 0.5, (blo.y + bhi.y) * 0.5);
        const Real bspan = std::max(bhi.x - blo.x, bhi.y - blo.y) + 10;
        const Real bx = 40 + 0 * cellW + 30, by = 118 + cellH;
        s.scale = 330.0 / std::max(bspan, Real(1));
        s.ox = bx + 165 - bmid.x * s.scale;
        s.oy = by + 165 - bmid.y * s.scale;
        if (!blocks.empty()) s.poly(blocks[big], "#f2eee5", "#c9c3b4", 1.2, "stroke-dasharray=\"5 4\"");
        for (const Poly2& l : dbg.lots) {
            if (l.empty() || blocks.empty()) continue;
            if (!engine::pointInPolygon(blocks[big], engine::centroid(l))) continue;
            s.poly(l, "#dfe7d5", "#7d9a6c", 1.0);
        }
        int here = 0;
        for (const engine::LotBuilding& lb : built) {
            if (blocks.empty() || lb.plan.size() < 3) continue;
            if (!engine::pointInPolygon(blocks[big], lb.site)) continue;
            s.poly(lb.plan, "#b9c0cc", "#232a36", 1.4);
            ++here;
        }
        snprintf(buf, sizeof buf, "%.0f m2 block, %d buildings", bestA, here);
        label(bx, by, "4 · one block, up close",
              "the grain the engine actually cuts", buf);
    }
    // --- 5: what the architect chose ----------------------------------------
    {
        const Real bx = 40 + 1 * cellW + 30, by = 118 + cellH;
        std::map<std::string, int> tally;
        for (const engine::LotBuilding& lb : built) tally[lb.recipe]++;
        std::vector<std::pair<int, std::string>> rows;
        for (const auto& [k, v] : tally) rows.push_back({v, k});
        std::sort(rows.rbegin(), rows.rend());
        int maxN = rows.empty() ? 1 : rows.front().first;
        for (std::size_t i = 0; i < rows.size() && i < 14; ++i) {
            const Real y = by + 24 + static_cast<Real>(i) * 22;
            const Real w = 150.0 * rows[i].first / std::max(1, maxN);
            fprintf(s.f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"13\" "
                         "fill=\"#c2c8d3\" stroke=\"#8a8578\" stroke-width=\"0.5\"/>\n",
                    bx + 130, y, std::max(w, Real(2)));
            s.textPx(bx + 124, y + 11, rows[i].second.c_str(), 10, "#4b5563", "end");
            snprintf(buf, sizeof buf, "%d", rows[i].first);
            s.textPx(bx + 286, y + 11, buf, 10, "#9aa1ad");
        }
        snprintf(buf, sizeof buf, "%zu distinct recipes across %zu lots",
                 tally.size(), built.size());
        label(bx, by, "5 · what the architect picked",
              "architectPick + the landmark planner, per lot", buf);
    }
    // --- 6: why lots did NOT build -------------------------------------------
    {
        const Real bx = 40 + 2 * cellW + 30, by = 118 + cellH;
        const struct { const char* n; int v; } rej[] = {
            {"occupancy roll", dbg.rejChance}, {"sliver (short side)", dbg.rejSliver},
            {"aspect (knife)", dbg.rejAspect}, {"fill ratio", dbg.rejFill},
            {"road clearance", dbg.rejClear},  {"box fallback", dbg.rejBox},
        };
        int worst = 1;
        for (const auto& r : rej) worst = std::max(worst, r.v);
        for (int i = 0; i < 6; ++i) {
            const Real y = by + 34 + i * 30;
            const Real w = 150.0 * rej[i].v / worst;
            fprintf(s.f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"16\" "
                         "fill=\"%s\" stroke=\"#8a8578\" stroke-width=\"0.5\"/>\n",
                    bx + 140, y, std::max(w, Real(2)),
                    rej[i].v > 0 ? "#e8b9ad" : "#e4e0d5");
            s.textPx(bx + 134, y + 13, rej[i].n, 10.5, "#4b5563", "end");
            snprintf(buf, sizeof buf, "%d", rej[i].v);
            s.textPx(bx + 298, y + 13, buf, 10.5, "#9aa1ad");
        }
        const int total = dbg.rejChance + dbg.rejSliver + dbg.rejAspect +
                          dbg.rejFill + dbg.rejClear + dbg.rejBox;
        snprintf(buf, sizeof buf, "%d of %zu parcels produced no building",
                 total, dbg.lots.size());
        label(bx, by, "6 · the engine's rejection counters",
              "the dials §17.1 proposes to delete by inverting the cut", buf);
    }
    svgClose(s);
    printf("  engine: %zu blocks, %zu parcels, %zu lot results\n",
           blocks.size(), dbg.lots.size(), built.size());
}

// ---------------------------------------------------------------------------
// Sheet 12 — BUILDABILITY: lots that can carry a building, and masses that can
// stand up. Six things the plan had wrong or unsaid.
// ---------------------------------------------------------------------------
static void sheetBuildable(const char* dir, uint32_t seed) {
    Svg s;
    const Real cellW = 440, cellH = 420;
    const int cols = 3, rows = 2;
    svgOpen(s, (std::string(dir) + "/buildable.svg").c_str(), cellW * cols + 60,
            cellH * rows + 180, "Lot Lab — buildability: lots that work, masses that stand");
    s.textPx(28, 68, "Today lots are cut on a rhythm and buildings that don't fit are rejected. "
                     "Inverted: a program declares its minimum rectangle, and only land that "
                     "holds one becomes a lot.", 12, "#6b7280");

    auto cell = [&](int i) {
        const int cx = i % cols, cy = i / cols;
        return std::make_pair(40 + cx * cellW + 30, Real(118 + cy * cellH));
    };
    auto label = [&](Real bx, Real by, const char* t, const char* d, const char* st) {
        s.textPx(bx, by + 262, t, 14, "#1d2430", "start", "600");
        s.textPx(bx, by + 279, d, 10.5, "#6b7280");
        s.textPx(bx, by + 294, st, 10, "#9aa1ad");
    };

    // A deliberately awkward block: a wedge, the shape that produces slivers.
    const Poly2 block{Vec2(0, 0), Vec2(58, 0), Vec2(46, 26), Vec2(0, 20)};

    // --- 1: uniform strips (today) — count what cannot build ---------------
    {
        auto [bx, by] = cell(0);
        s.scale = 3.4; s.ox = bx; s.oy = by + 20;
        s.poly(block, "#f2eee5", "#c9c3b4", 1.0, "stroke-dasharray=\"5 4\"");
        int bad = 0, good = 0;
        for (int k = 0; k < 7; ++k) {                    // uniform rhythm
            const Real x0 = 58.0 * k / 7.0, x1 = 58.0 * (k + 1) / 7.0;
            Poly2 lot = clipToLot(rectPoly(Rect(x0, -2, x1, 30)), block);
            if (lot.size() < 3) continue;
            Real w, d;
            maxInscribedRect(lot, w, d);
            const bool ok = std::min(w, d) >= 9.0 && engine::area(lot) >= 150.0;
            (ok ? good : bad)++;
            s.poly(lot, ok ? "#cfe1c2" : "#efc9be", "#8a8578", 1.2);
        }
        char buf[128];
        snprintf(buf, sizeof buf, "%d of 7 lots cannot carry a house", bad);
        label(bx, by, "1 · uniform rhythm (today)",
              "cut first, reject later — red lots become slivers", buf);
    }
    // --- 2: program-driven parcelling ---------------------------------------
    {
        auto [bx, by] = cell(1);
        s.scale = 3.4; s.ox = bx; s.oy = by + 20;
        s.poly(block, "#f2eee5", "#c9c3b4", 1.0, "stroke-dasharray=\"5 4\"");
        // Walk the frontage placing lots only where the minimum rectangle fits;
        // widen until it does, and hand what is left to open space.
        const Real minW = 11.0, minD = 12.0;
        Real x = 0;
        int placed = 0;
        Real leftover = 0;
        while (x < 58.0 - 1.0) {
            Real w = minW;
            Poly2 lot;
            bool ok = false;
            for (; w <= 26.0; w += 1.0) {
                lot = clipToLot(rectPoly(Rect(x, -2, std::min(x + w, Real(58)), 30)), block);
                if (lot.size() < 3) continue;
                Real iw, id;
                maxInscribedRect(lot, iw, id);
                if (std::min(iw, id) >= std::min(minW, minD) * 0.82 &&
                    engine::area(lot) >= 190.0) { ok = true; break; }
            }
            if (!ok) { leftover += 58.0 - x; break; }
            s.poly(lot, "#cfe1c2", "#4d6b43", 1.4);
            Real iw, id;
            maxInscribedRect(lot, iw, id);
            const engine::OBB2 ob = engine::orientedBoundingBox(lot);
            Vec2 c = engine::centroid(lot);
            if (!engine::pointInPolygon(lot, c)) c = ob.center;
            s.poly(Poly2{c - ob.axis[0] * iw * 0.5 - ob.axis[1] * id * 0.5,
                         c + ob.axis[0] * iw * 0.5 - ob.axis[1] * id * 0.5,
                         c + ob.axis[0] * iw * 0.5 + ob.axis[1] * id * 0.5,
                         c - ob.axis[0] * iw * 0.5 + ob.axis[1] * id * 0.5},
                   "none", "#4d6b43", 0.8, "stroke-dasharray=\"3 3\"");
            ++placed;
            x += w;
        }
        if (leftover > 1.0) {
            Poly2 rest = clipToLot(rectPoly(Rect(58.0 - leftover, -2, 60, 30)), block);
            if (rest.size() >= 3) s.poly(rest, "#dbe4ee", "#7c86a0", 1.2);
        }
        char buf[128];
        snprintf(buf, sizeof buf, "%d lots, every one holds its minimum rectangle", placed);
        label(bx, by, "2 · program-driven (proposed)",
              "dashed = the minimum rectangle the program needs", buf);
    }
    // --- 3: minimum plate vs height -----------------------------------------
    {
        auto [bx, by] = cell(2);
        const Real plates[4][2] = {{12, 14}, {20, 22}, {30, 32}, {44, 46}};
        for (int k = 0; k < 4; ++k) {
            const Real w = plates[k][0], d = plates[k][1];
            const int cap = storeyCapFor(w, d);
            const Real px = bx + k * 92, py = by + 200;
            s.scale = 1.5;
            s.ox = px; s.oy = by + 40;
            s.poly(rectPoly(Rect(0, 0, w, d)), "#dfe1e6", "#8a8578", 1.0);
            const Real h = std::min(Real(150), cap * 2.4);
            fprintf(s.f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"20\" height=\"%.1f\" "
                         "fill=\"#c2c8d3\" stroke=\"#232a36\" stroke-width=\"1\"/>\n",
                    px + 24, py - h, h);
            char buf[64];
            snprintf(buf, sizeof buf, "%.0fx%.0f", w, d);
            s.textPx(px, py + 14, buf, 9.5, "#6b7280");
            snprintf(buf, sizeof buf, "%d fl", cap);
            s.textPx(px, py + 26, buf, 9.5, "#9aa1ad");
        }
        label(bx, by, "3 · the plate caps the height",
              "lifts, core and structure scale with the storeys they serve",
              "one rule, not a guess repeated in six tower recipes");
    }
    // --- 4: the SUPPORT rule (cantilevers) ----------------------------------
    {
        auto [bx, by] = cell(3);
        s.scale = 4.2; s.ox = bx + 90; s.oy = by + 110;
        const Poly2 base = rectPoly(Rect(-11, -8, 11, 8));
        const Poly2 upper = rectPoly(Rect(-5, -14, 17, 3));      // overhangs
        auto lap = polyBool({upper}, {base}, BOp::Intersect);
        Real over = 0;
        for (const Poly2& p : lap) over += engine::area(p);
        const Real frac = over / std::max(Real(1), engine::area(upper));
        s.poly(base, "#dfe1e6", "#8a8578", 1.4, "stroke-dasharray=\"5 4\"");
        for (const Poly2& p : polyBool({upper}, {base}, BOp::Subtract))
            s.poly(p, "#e8c9a8", "#a5773f", 1.2);                // unsupported
        for (const Poly2& p : lap) s.poly(p, "#b9c0cc", "#232a36", 1.6);
        char buf[128];
        snprintf(buf, sizeof buf, "support = %.0f%% of the upper plate", frac * 100);
        label(bx, by, "4 · cantilever needs a SUPPORT rule",
              "containment was too strong — orange is unsupported", buf);
    }
    // --- 5: the plan quality gate -------------------------------------------
    {
        auto [bx, by] = cell(4);
        s.scale = 4.6; s.ox = bx + 100; s.oy = by + 76;
        // The defect: a notch that would ride the whole shaft.
        const Poly2 bad{Vec2(-14, -9), Vec2(14, -9), Vec2(14, 9), Vec2(2.4, 9),
                        Vec2(1.2, -3), Vec2(0.0, 9), Vec2(-14, 9)};
        const PlanDefect d = planQuality(bad);
        s.poly(bad, "#efc9be", "#a5433a", 1.6);
        const Poly2 fixed{Vec2(-14, -9), Vec2(14, -9), Vec2(14, 9), Vec2(-14, 9)};
        s.ox = bx + 100; s.oy = by + 190;
        s.poly(fixed, "#cfe1c2", "#4d6b43", 1.6);
        char buf[160];
        snprintf(buf, sizeof buf, "rejected: %s (%.1f)", d.why, d.value);
        label(bx, by, "5 · plan quality is an invariant",
              "a base defect is amplified up every storey above it", buf);
    }
    // --- 6: rim blocks can be big --------------------------------------------
    {
        auto [bx, by] = cell(5);
        s.scale = 1.9; s.ox = bx; s.oy = by + 18;
        // interior block: enclosed on all sides, fine grain
        s.poly(rectPoly(Rect(0, 0, 46, 30)), "#f2eee5", "#c9c3b4", 1.0,
               "stroke-dasharray=\"5 4\"");
        for (int k = 0; k < 4; ++k)
            s.poly(rectPoly(Rect(k * 11.5, 0, (k + 1) * 11.5, 14)), "#cfe1c2", "#8a8578", 1.0);
        s.textPx(bx, by + 92, "enclosed block: fine grain", 9.5, "#6b7280");
        // rim block: open on the far side, so depth is not bounded
        s.oy = by + 116;
        s.poly(rectPoly(Rect(0, 0, 46, 46)), "#f2eee5", "#c9c3b4", 1.0,
               "stroke-dasharray=\"5 4\"");
        s.poly(rectPoly(Rect(0, 0, 26, 44)), "#dbe4ee", "#4a6a8a", 1.4);
        s.poly(rectPoly(Rect(27, 0, 46, 24)), "#dbe4ee", "#4a6a8a", 1.4);
        s.textPx(bx, by + 214, "rim block: campus / industrial parcels", 9.5, "#6b7280");
        label(bx, by, "6 · the city edge parcels coarse",
              "no far-side street means depth is unbounded",
              "university, office park, big box, works");
    }
    (void)seed;
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 11 — PORTING THE OLD RECIPES. Left: what the current architect can
// actually produce for that recipe. Right: what the same recipe becomes once a
// plan can be several masses, a stack can have per-band heights, and cladding
// is decoupled from fenestration. Six of the weakest cases.
// ---------------------------------------------------------------------------
static void sheetPort(const char* dir, uint32_t seed) {
    Svg s;
    const Real cellW = 440, cellH = 400;
    const int cols = 3, rows = 2;
    svgOpen(s, (std::string(dir) + "/port.svg").c_str(), cellW * cols + 60,
            cellH * rows + 180, "Lot Lab — porting the existing recipes, and what it buys");
    s.textPx(28, 68, "Left of each pair: what today's recipe can express — one prism, one window "
                     "style, one floor height. Right: the same recipe as a table, using masses, "
                     "bands and per-wall fenestration.", 12, "#6b7280");

    auto pair = [&](int i, const char* title, const char* was, const char* now,
                    const Poly2& oldPlan,
                    const std::vector<Poly2>& newPlans,
                    const std::vector<Opening>& newOps,
                    const Poly2& opsOn, Real span) {
        const int cx = i % cols, cy = i / cols;
        const Real bx = 40 + cx * cellW + 26, by = 118 + cy * cellH;
        const Real half = (cellW - 90) * 0.5;
        const Real sc = std::min(half / span, 190.0 / span);
        // OLD — a thin grey outline, deliberately unglamorous
        s.scale = sc;
        s.ox = bx + half * 0.5;
        s.oy = by + 100;
        s.poly(oldPlan, "#e9e6de", "#a8a29a", 1.4);
        s.textPx(bx, by + 200, "today", 10, "#a8a29a", "start", "600");
        // NEW — the ported table's result
        s.ox = bx + half + 26 + half * 0.5;
        if (!newOps.empty() && opsOn.size() >= 3) drawBlueprint(s, opsOn, 0.32, newOps);
        for (const Poly2& p : newPlans) s.poly(p, "#b9c0cc", "#232a36", 1.8);
        s.textPx(bx + half + 26, by + 200, "ported", 10, "#4b5563", "start", "600");

        s.textPx(bx, by + 244, title, 14, "#1d2430", "start", "600");
        s.textPx(bx, by + 261, was, 10.5, "#9aa1ad");
        s.textPx(bx, by + 277, now, 10.5, "#4b5563");
    };

    // 1 — office_park: the clearest failure. Wants two blocks; gets one box.
    {
        const Poly2 oldP = rectPoly(Rect(-13, -9, 13, 9));
        PlanCtx a; a.envelope = Rect(-26, -14, 26, 14);
        seedMass(a, Rect(-25, -3, -6, 13));
        court(a, -19, 5, -12, 14);
        a.rs.unite(Rect(3, -2, 24, 12));
        a.rs.unite(Rect(-6, 3, 3, 7));                 // the link
        std::vector<Region> rs = planFinish(a);
        std::vector<Poly2> np;
        for (const Region& r : rs) np.push_back(r.outer);
        pair(0, "office_park", "one prism — the name is a lie",
             "two blocks, atrium notch, a link bar", oldP, np, {}, {}, 56);
    }
    // 2 — church: floors = 0 plus a tall ground storey, faking a nave.
    {
        const Poly2 oldP = rectPoly(Rect(-8, -13, 8, 13));
        PlanCtx a; a.envelope = Rect(-14, -16, 14, 16);
        seedMass(a, Rect(-6, -12, 6, 10));             // nave
        a.rs.unite(Rect(-11, -8, -6, 6));              // aisle
        a.rs.unite(Rect(6, -8, 11, 6));                // aisle
        a.rs.unite(Rect(-4, 10, 4, 15));               // tower base
        std::vector<Region> rs = planFinish(a);
        std::vector<Poly2> np;
        for (const Region& r : rs) np.push_back(r.outer);
        pair(1, "church", "one box, floors = 0 as a hack",
             "nave + two aisles + tower base: 4 masses", oldP, np, {}, {}, 34);
    }
    // 3 — strip_mall: a long linear bar of units with a canopy.
    {
        const Poly2 oldP = rectPoly(Rect(-16, -8, 16, 8));
        PlanCtx a; a.envelope = Rect(-30, -12, 30, 12);
        seedMass(a, Rect(-26, -6, 20, 6));
        a.rs.unite(Rect(20, -10, 28, 8));              // the end anchor
        a.rs.unite(Rect(-26, -8, 20, -6));             // the canopy line
        std::vector<Region> rs = planFinish(a);
        std::vector<Poly2> np;
        for (const Region& r : rs) np.push_back(r.outer);
        pair(2, "strip_mall", "a square box called a strip",
             "long bar + end anchor + canopy run", oldP, np, {}, {}, 62);
    }
    // 4 — hotel: same floor height as an office; podium never expressed.
    {
        const Poly2 oldP = rectPoly(Rect(-11, -9, 11, 9));
        const Real bx = 40 + 0 * cellW + 26, by = 118 + cellH;
        std::vector<Poly2> podium{rectPoly(Rect(-15, -11, 15, 11))};
        std::vector<Poly2> tower =
            polyBool({rectPoly(Rect(-9, -7, 9, 7))}, podium, BOp::Intersect);
        std::vector<Poly2> np = podium;
        np.insert(np.end(), tower.begin(), tower.end());
        pair(3, "hotel", "3.2 m floors, no podium — an office in disguise",
             "4.6 m lobby band + 3.1 m room band on a podium", oldP, np, {}, {}, 40);
        // the height story needs a section
        std::vector<std::vector<Poly2>> lv;
        for (int k = 0; k < 4; ++k) lv.push_back(podium);
        for (int k = 0; k < 11; ++k) lv.push_back(tower);
        drawSection(s, lv, -17, 17, 0, bx + 372, by + 196, 2.1, 190.0 / 15.0, "#c2c8d3");
        s.textPx(bx + 372, by + 208, "band section", 9.5, "#9aa1ad");
    }
    // 5 — loft_conversion: dress() forces arched sash onto brick.
    {
        const Poly2 plan = rectPoly(Rect(0, 0, 17, 11));
        std::vector<WallRole> roles{WallRole::Street, WallRole::Party,
                                    WallRole::Rear, WallRole::Party};
        FenRecipe rec{"loft", "", {
            {WallRole::Street, Fen::Plate, 0, 0, 1.1, 0.7, 3.1, true},
            {WallRole::Rear,   Fen::Plate, 0, 0, 1.1, 0.7, 3.1, false},
            {WallRole::Party,  Fen::Blank}}, 0.0};
        pair(4, "loft_conversion", "brick forced to wear round-arched sash",
             "brick walls, big steel plate glazing", plan, {},
             fenestrate(plan, roles, rec), plan, 22);
    }
    // 6 — bungalow vs craftsman: two recipes, one shape, one differing bool.
    {
        const Poly2 oldP = rectPoly(Rect(0, 0, 13, 9));
        PlanCtx a; a.envelope = Rect(-2, -4, 16, 13);
        seedMass(a, Rect(0, 0, 13, 9));
        a.rs.unite(Rect(0, 9, 8, 13));                 // the rear ell
        a.rs.unite(Rect(-1.6, -2.6, 13, 0.1));         // the wrapping porch
        std::vector<Region> rs = planFinish(a);
        std::vector<Poly2> np;
        for (const Region& r : rs) np.push_back(r.outer);
        pair(5, "craftsman (vs bungalow)", "same rectangle; one bool apart",
             "L-plan with a rear ell and a wrapping porch", oldP, np, {}, {}, 24);
    }
    (void)seed;
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 10 — SILHOUETTES: profile curves, merging lofts, holes, and a pen plan.
// ---------------------------------------------------------------------------
static void sheetSilhouette(const char* dir, uint32_t seed) {
    Svg s;
    const Real cellW = 430, cellH = 470;
    const int cols = 3, rows = 2;
    svgOpen(s, (std::string(dir) + "/silhouette.svg").c_str(), cellW * cols + 60,
            cellH * rows + 180, "Lot Lab — profile, loft and twist are different knobs");
    s.textPx(28, 68, "A Gherkin is ONE plan with a PROFILE curve and no loft. A pyramid is a "
                     "linear profile. An arch is a loft that MERGES going up. Twist is a "
                     "fourth knob, and should be rationed (panel 6).", 12, "#6b7280");

    auto panel = [&](int i, const char* title, const char* sub, const char* stat,
                     const std::vector<std::vector<Poly2>>& levels,
                     Real span, Real yMid) {
        const int cx = i % cols, cy = i / cols;
        const Real bx = 40 + cx * cellW + 30, by = 118 + cy * cellH;
        // top: the storey plans, superimposed
        const Real psc = 150.0 / span;
        s.scale = psc;
        s.ox = bx + 96;
        s.oy = by + 84;
        const std::size_t stride = std::max<std::size_t>(1, levels.size() / 6);
        for (std::size_t L = 0; L < levels.size(); ++L) {
            if (L % stride && L + 1 != levels.size()) continue;
            char col[16];
            const int g = 32 + static_cast<int>(120.0 * L / std::max<std::size_t>(1, levels.size() - 1));
            snprintf(col, sizeof col, "#%02x%02x%02x", g, g + 8, g + 20);
            for (const Poly2& p : levels[L])
                s.poly(p, "none", col, L == 0 || L + 1 == levels.size() ? 1.8 : 0.9);
        }
        s.textPx(bx + 190, by + 18, "storey plans", 10, "#9aa1ad");
        // bottom: the centre section, built from those same plans
        const Real ssc = 150.0 / span;
        drawSection(s, levels, -span * 0.5, span * 0.5, yMid, bx + 210,
                    by + 300, ssc, 300.0 / levels.size(), "#c2c8d3");
        s.textPx(bx + 210, by + 318, "centre section", 10, "#9aa1ad");
        s.textPx(bx, by + 356, title, 14, "#1d2430", "start", "600");
        s.textPx(bx, by + 373, sub, 11, "#6b7280");
        s.textPx(bx, by + 388, stat, 10, "#9aa1ad");
    };

    const int NL = 16;
    auto build = [&](const std::vector<Poly2>& A, const std::vector<Poly2>* B,
                     const Profile& prof, Real twist) {
        std::vector<std::vector<Poly2>> lv;
        for (int i = 0; i < NL; ++i) {
            const Real t = static_cast<Real>(i) / (NL - 1);
            lv.push_back(storeyPlan(A, B, t, prof, twist * t));
        }
        return lv;
    };

    // 1 — the Gherkin: one plan, a swell profile, NO loft, NO twist
    {
        std::vector<Poly2> A{regularPoly(Vec2(0, 0), 15, 28, 0)};
        panel(0, "1 · profile curve (Gherkin)", "one plan, swell-then-nose; no loft, no twist",
              "profile = sin(pi(0.20+0.78t))^0.85", build(A, nullptr, profileSwell(), 0), 34, 0);
    }
    // 2 — pyramid
    {
        std::vector<Poly2> A{rectPoly(Rect(-16, -16, 16, 16))};
        panel(1, "2 · linear taper (pyramid)", "same machinery, a straight-line profile",
              "profile = 1 - 0.97t", build(A, nullptr, profileTaper(0.03), 0), 36, 0);
    }
    // 3 — the ARCH: two legs merging into one span going up
    {
        std::vector<Poly2> A{rectPoly(Rect(-17, -6, -7, 6)), rectPoly(Rect(7, -6, 17, 6))};
        std::vector<Poly2> B{rectPoly(Rect(-17, -6, 17, 6))};
        panel(2, "3 · loft that MERGES (arch)", "two legs at grade, one span at the top",
              "topology merge — the mirror of the slab->towers split",
              build(A, &B, profileConstant(), 0), 40, 0);
    }
    // 4 — donut: a plan with a hole, held all the way up
    {
        std::vector<Poly2> ring = polyBool({regularPoly(Vec2(0, 0), 17, 32, 0)},
                                           {regularPoly(Vec2(0, 0), 8.5, 32, 0)},
                                           BOp::Subtract);
        panel(3, "4 · a hole, held (donut)", "Shape2 carries the hole; nothing else changes",
              "the void is a loop, not a special case",
              build(ring, nullptr, profileConstant(), 0), 38, 0);
    }
    // 5 — a PEN-drawn plan, then corner cuts on an irregular polygon
    {
        Pen pen;
        pen.moveTo(-14, -10).forward(0);
        pen.dir = 0;
        pen.pts.clear();
        pen.cur = Vec2(-14, -10);
        pen.pts.push_back(pen.cur);
        pen.forward(12).turn(-90).forward(6).turn(90).forward(9)
           .turn(90).forward(7).turn(-90).forward(7).turn(90)
           .forward(13).turn(90).forward(28).turn(90).forward(12);
        Poly2 p = pen.close();
        std::vector<WallRole> roles(p.size(), WallRole::Street);
        // Corner cuts on an IRREGULAR plan — the op is per-vertex, so it never
        // cared that the earlier demo happened to be a rectangle.
        for (int k = 0; k < 3; ++k) {
            std::size_t v = (k * 3 + 1) % p.size();
            p = chamferCorner(p, roles, v, 2.6);
        }
        std::vector<Poly2> A{p};
        panel(4, "5 · pen-drawn plan, corner cuts", "forward/turn walk, then chamfer 3 vertices",
              "the chamfer op is per-vertex — shape-agnostic",
              build(A, nullptr, profileTaper(0.55), 0), 42, -2);
    }
    // 6 — RARITY: signature massing is a quota, not a probability
    {
        const int cx = 2, cy = 1;
        const Real bx = 40 + cx * cellW + 30, by = 118 + cy * cellH;
        std::vector<Poly2> box{rectPoly(Rect(-9, -9, 9, 9))};
        const Real bw = 34, gap = 6;
        for (int k = 0; k < 9; ++k) {
            const bool signature = (k == 5);
            const Profile pr = signature ? profileSwell() : profileTaper(0.86);
            std::vector<std::vector<Poly2>> lv;
            const int nl = signature ? 18 : 8 + (k * 5) % 7;
            for (int i = 0; i < nl; ++i)
                lv.push_back(storeyPlan(box, nullptr, static_cast<Real>(i) / (nl - 1), pr,
                                        signature ? 0.55 * i / (nl - 1) : 0));
            drawSection(s, lv, -11, 11, 0, bx + k * (bw + gap) * 0.62, by + 250,
                        1.5, 250.0 / 18.0, signature ? "#b48a5c" : "#c9ccd3");
        }
        s.textPx(bx, by + 300, "6 · signature massing is a QUOTA", 14, "#1d2430", "start", "600");
        s.textPx(bx, by + 317, "one signature tower per city, PLACED on the best site",
                 11, "#6b7280");
        s.textPx(bx, by + 332, "the same planner that places one courthouse, not a per-lot roll",
                 10, "#9aa1ad");
    }
    (void)seed;
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 9 — CORNERS AND STACKS: where the context facts come from.
// ---------------------------------------------------------------------------
static void sheetCorner(const char* dir, uint32_t seed) {
    Svg s;
    const Real cellW = 430, cellH = 420;
    const int cols = 3, rows = 2;
    svgOpen(s, (std::string(dir) + "/corner.svg").c_str(), cellW * cols + 60,
            cellH * rows + 180, "Lot Lab — corner lots, and where floor heights come from");
    s.textPx(28, 68, "Cornerness is a fact about the PARCEL'S PLACE IN THE BLOCK, not about the "
                     "building. The parceller tags each edge by what lies across it; everything "
                     "downstream selects on the tag.", 12, "#6b7280");

    // --- the block and its parcels ----------------------------------------
    const Real BW = 62, BD = 42, D = 14;      // block size, lot depth
    const Poly2 block{Vec2(0, 0), Vec2(BW, 0), Vec2(BW, BD), Vec2(0, BD)};
    auto onBlock = [&](const Vec2& p) {
        return p.x < 0.01 || p.y < 0.01 || p.x > BW - 0.01 || p.y > BD - 0.01;
    };
    auto onCore = [&](const Vec2& p) {
        return p.x > D - 0.01 && p.x < BW - D + 0.01 &&
               p.y > D - 0.01 && p.y < BD - D + 0.01;
    };
    std::vector<Poly2> lots;
    lots.push_back(rectPoly(Rect(0, 0, D, D)));                    // 4 corners
    lots.push_back(rectPoly(Rect(BW - D, 0, BW, D)));
    lots.push_back(rectPoly(Rect(BW - D, BD - D, BW, BD)));
    lots.push_back(rectPoly(Rect(0, BD - D, D, BD)));
    for (int i = 0; i < 3; ++i) {                                  // top + bottom runs
        const Real x0 = D + (BW - 2 * D) * i / 3.0, x1 = D + (BW - 2 * D) * (i + 1) / 3.0;
        lots.push_back(rectPoly(Rect(x0, 0, x1, D)));
        lots.push_back(rectPoly(Rect(x0, BD - D, x1, BD)));
    }
    lots.push_back(rectPoly(Rect(0, D, D, BD - D)));               // side runs
    lots.push_back(rectPoly(Rect(BW - D, D, BW, BD - D)));

    auto roleColor = [](WallRole r) {
        switch (r) {
            case WallRole::Street: return "#c9553d";
            case WallRole::Rear:   return "#5d8a4c";
            case WallRole::Party:  return "#7c86a0";
            default:               return "#999";
        }
    };

    // --- 1: the tagged block -----------------------------------------------
    {
        s.scale = std::min((cellW - 90) / BW, 250.0 / BD);
        s.ox = 40 + 34;
        s.oy = 118 + 20;
        s.poly(block, "#f2eee5", "#c9c3b4", 1.0, "stroke-dasharray=\"5 4\"");
        s.poly(rectPoly(Rect(D, D, BW - D, BD - D)), "#e7ecdf", "#c9c3b4", 1.0);
        int corners = 0;
        for (const Poly2& L : lots) {
            const LotContext c = classifyLot(L, onBlock, onCore);
            const bool isCorner = c.shape == LotShape::Corner;
            if (isCorner) ++corners;
            s.poly(L, isCorner ? "#f6e3c8" : "#faf8f3", "none", 0);
            for (std::size_t i = 0; i < L.size(); ++i)
                s.line(L[i], L[(i + 1) % L.size()], roleColor(c.roles[i]), 2.2);
        }
        char buf[160];
        s.textPx(40 + 34, 118 + BD * s.scale + 60, "1 · the parceller tags every edge",
                 14, "#1d2430", "start", "600");
        snprintf(buf, sizeof buf,
                 "red = street · green = block core (rear) · blue = neighbour (party)");
        s.textPx(40 + 34, 118 + BD * s.scale + 77, buf, 10.5, "#6b7280");
        snprintf(buf, sizeof buf, "%d lots · %d classify as CORNER (two adjacent street edges)",
                 static_cast<int>(lots.size()), corners);
        s.textPx(40 + 34, 118 + BD * s.scale + 92, buf, 10, "#9aa1ad");
    }

    // --- 2: the four contexts ----------------------------------------------
    {
        const Real bx = 40 + cellW + 34, by = 118 + 20;
        struct Ctx { const char* n; std::vector<int> street; };
        const Ctx kinds[4] = {{"mid-block", {0}}, {"corner", {0, 1}},
                              {"through", {0, 2}}, {"island", {0, 1, 2, 3}}};
        for (int k = 0; k < 4; ++k) {
            const Real ox = bx + (k % 2) * 160, oy = by + (k / 2) * 120;
            s.scale = 5.2;
            s.ox = ox;
            s.oy = oy;
            const Poly2 L = rectPoly(Rect(0, 0, 16, 12));
            s.poly(L, "#f6e3c8", "none", 0);
            for (std::size_t i = 0; i < 4; ++i) {
                const bool st = std::find(kinds[k].street.begin(), kinds[k].street.end(),
                                          static_cast<int>(i)) != kinds[k].street.end();
                s.line(L[i], L[(i + 1) % 4], st ? "#c9553d" : "#7c86a0", st ? 2.6 : 1.6);
            }
            s.textPx(ox, oy + 12 * s.scale + 16, kinds[k].n, 11, "#4b5563", "start", "600");
        }
        s.textPx(bx, by + 250, "2 · four contexts, one test", 14, "#1d2430", "start", "600");
        s.textPx(bx, by + 267, "adjacent street edges = corner; opposite = through",
                 10.5, "#6b7280");
        s.textPx(bx, by + 282, "computable at parcel time, exact, no heuristics", 10, "#9aa1ad");
    }

    // --- 3: the chamfered corner entrance ----------------------------------
    {
        const Real bx = 40 + 2 * cellW + 34, by = 118 + 20;
        Poly2 lot = rectPoly(Rect(0, 0, 15, 13));
        std::vector<WallRole> roles{WallRole::Street, WallRole::Party,
                                    WallRole::Rear, WallRole::Street};
        // The corner is vertex 0 — where edge 3 (street) meets edge 0 (street).
        lot = chamferCorner(lot, roles, 0, 3.6);
        FenRecipe rec{"corner shop", "", {
            {WallRole::Street, Fen::Storefront, 0, 0, 0, 0.25, 2.9, false},
            {WallRole::Party,  Fen::Blank},
            {WallRole::Rear,   Fen::Punched, 3.0, 1.0, 0, 1.2, 2.4, false}}, 0.0};
        std::vector<Opening> ops = fenestrate(lot, roles, rec);
        // The door goes on the CHAMFER — addressable because it carries a tag.
        for (std::size_t i = 0; i < lot.size(); ++i) {
            if (i != 0) continue;
            const Real len = (lot[1] - lot[0]).length();
            ops.erase(std::remove_if(ops.begin(), ops.end(),
                                     [](const Opening& o) { return o.edge == 0; }),
                      ops.end());
            ops.push_back({0, len * 0.5 - 1.1, len * 0.5 + 1.1, true, -1, 0.0, 2.4});
        }
        s.scale = 15.0;
        s.ox = bx + 10;
        s.oy = by + 10;
        drawBlueprint(s, lot, 0.32, ops);
        for (std::size_t i = 0; i < lot.size(); ++i)
            s.line(lot[i], lot[(i + 1) % lot.size()], roleColor(roles[i]), 1.8);
        s.textPx(bx, by + 13 * s.scale + 74, "3 · diagonal corner entrance", 14,
                 "#1d2430", "start", "600");
        s.textPx(bx, by + 13 * s.scale + 91,
                 "chamfer(vertex) makes the edge; the tag makes it addressable",
                 10.5, "#6b7280");
        s.textPx(bx, by + 13 * s.scale + 106,
                 "door selector: on = \"edge:corner_chamfer\"", 10, "#9aa1ad");
    }

    // --- 4: same lot, different recipe answer ------------------------------
    {
        const Real bx = 40 + 34, by = 118 + cellH;
        const Poly2 lot = rectPoly(Rect(0, 0, 15, 13));
        std::vector<WallRole> roles{WallRole::Street, WallRole::Party,
                                    WallRole::Rear, WallRole::Street};
        FenRecipe rec{"modern corner", "", {
            {WallRole::Street, Fen::Plate, 0, 0, 1.6, 0.45, 2.7, true},
            {WallRole::Party,  Fen::Blank},
            {WallRole::Rear,   Fen::Plate, 0, 0, 1.4, 0.45, 2.7, false}}, 2.6, 0.45, 2.7};
        const std::vector<Opening> ops = fenestrate(lot, roles, rec);
        s.scale = 15.0;
        s.ox = bx + 10;
        s.oy = by + 30;
        drawBlueprint(s, lot, 0.32, ops);
        for (std::size_t i = 0; i < lot.size(); ++i)
            s.line(lot[i], lot[(i + 1) % lot.size()], roleColor(roles[i]), 1.8);
        s.textPx(bx, by + 13 * s.scale + 94, "4 · same corner, other recipe", 14,
                 "#1d2430", "start", "600");
        s.textPx(bx, by + 13 * s.scale + 111,
                 "corner GLAZING instead of a chamfer — one context, two answers",
                 10.5, "#6b7280");
        s.textPx(bx, by + 13 * s.scale + 126,
                 "the context is a fact; the response is a recipe choice", 10, "#9aa1ad");
    }

    // --- 5: storey bands ----------------------------------------------------
    {
        const Real bx = 40 + cellW + 34, by = 118 + cellH + 20;
        const StoreyBand kStack[] = {
            {BandRole::Retail, 1, 5.4, "podium", nullptr, 0},
            {BandRole::Lobby,  1, 4.2, "podium", nullptr, 0},
            {BandRole::Parking, 3, 3.0, "podium", nullptr, 0},
            {BandRole::Office, 9, 3.9, "shaft", "shaft_top", 0.30},
            {BandRole::Plant,  1, 4.6, "shaft", nullptr, 0},
            {BandRole::Crown,  2, 5.0, "crown", nullptr, 0},
        };
        const Real sc = 4.4, W = 120;
        Real y = by + 250;
        char buf[160];
        for (const StoreyBand& b : kStack) {
            const Real h = b.storeys * b.height * sc;
            fprintf(s.f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" "
                         "fill=\"%s\" stroke=\"#232a36\" stroke-width=\"1.1\"/>\n",
                    bx, y - h, W, h,
                    b.role == BandRole::Crown ? "#aeb6c4"
                        : b.role == BandRole::Plant ? "#cfd3da"
                        : b.role == BandRole::Office ? "#c2c8d3" : "#dfe1e6");
            for (int k = 1; k < b.storeys; ++k)          // floor lines
                fprintf(s.f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" "
                             "stroke=\"#9aa1ad\" stroke-width=\"0.5\"/>\n",
                        bx, y - h * k / b.storeys, bx + W, y - h * k / b.storeys);
            snprintf(buf, sizeof buf, "%s  %d x %.1f m", bandName(b.role), b.storeys, b.height);
            s.textPx(bx + W + 10, y - h * 0.5 + 4, buf, 10, "#4b5563");
            y -= h;
        }
        s.textPx(bx, by + 288, "5 · height belongs to the BAND", 14, "#1d2430", "start", "600");
        s.textPx(bx, by + 305, "one tower: 5.4 m retail, 4.2 lobby, 3.0 parking, 3.9 office…",
                 10.5, "#6b7280");
        s.textPx(bx, by + 320, "floorHeight + groundHeight can express exactly two of these",
                 10, "#9aa1ad");
    }

    // --- 6: the loft is a band property -------------------------------------
    {
        const Real bx = 40 + 2 * cellW + 34, by = 118 + cellH + 20;
        std::vector<Poly2> A{rectPoly(Rect(-13, -9, 13, 9))};
        std::vector<Poly2> B{regularPoly(Vec2(0, 0), 7.5, 32, 0)};
        s.scale = 7.0;
        s.ox = bx + 130;
        s.oy = by + 120;
        for (int k = 0; k <= 6; ++k) {
            const Real t = k / 6.0;
            char col[16];
            snprintf(col, sizeof col, "#%02x%02x%02x", 35 + k * 13, 42 + k * 13, 54 + k * 13);
            for (const Poly2& p : loftPlans(A, B, t, t * 0.30, 0.16))
                s.poly(p, "none", col, k == 0 || k == 6 ? 2.0 : 1.0);
        }
        s.textPx(bx, by + 288, "6 · the loft is band data", 14, "#1d2430", "start", "600");
        s.textPx(bx, by + 305, "{ plan=\"shaft\", loft_to=\"shaft_top\", twist=0.30 }",
                 10.5, "#6b7280");
        s.textPx(bx, by + 320, "9 office floors resolve to 9 storey plans", 10, "#9aa1ad");
    }
    (void)seed;
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 8 — ONE PLAN, FIVE RECIPES. Same geometry, same code path; only the
// fenestration table differs. If this sheet varies, the data/code line holds.
// ---------------------------------------------------------------------------
static void sheetRecipes(const char* dir, uint32_t seed) {
    // The elevation is where the storey's half of the fact becomes visible:
    // the plan owns the span along the wall, the storey owns sill and head.
    auto elevation = [](Svg& s, Real px, Real py, Real len, Real storey,
                        const std::vector<Opening>& ops, std::size_t edge,
                        Real sc, bool mullions) {
        const Real W = len * sc, H = storey * sc;
        fprintf(s.f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" "
                     "fill=\"#e4e0d5\" stroke=\"#232a36\" stroke-width=\"1.4\"/>\n",
                px, py - H, W, H);
        fprintf(s.f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" "
                     "stroke=\"#8a8578\" stroke-width=\"1.0\"/>\n",
                px - 8, py, px + W + 8, py);                   // ground line
        for (const Opening& o : ops) {
            if (o.edge != edge) continue;
            const Real x0 = px + o.s0 * sc, x1 = px + o.s1 * sc;
            const Real y0 = py - o.head * sc, y1 = py - o.sill * sc;
            fprintf(s.f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" "
                         "fill=\"%s\" stroke=\"#232a36\" stroke-width=\"1.0\"/>\n",
                    x0, y0, x1 - x0, y1 - y0, o.door ? "#6b5a45" : "#9fc4dd");
            if (mullions && x1 - x0 > 20) {                    // curtain grid
                for (Real x = x0 + 14; x < x1 - 6; x += 14)
                    fprintf(s.f, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" "
                                 "stroke=\"#5b6474\" stroke-width=\"0.9\"/>\n", x, y0, x, y1);
            }
        }
    };

    // ---- THE DATA. Everything that distinguishes these five buildings is in
    // this array; not one line of geometry code below knows they exist. ----
    const FenRecipe kRecipes[] = {
        {"victorian terrace", "punched sash on a 3.0 m module; party walls blank",
         {{WallRole::Street, Fen::Punched, 3.0, 1.05, 0, 0.9, 2.5, true},
          {WallRole::Party,  Fen::Blank},
          {WallRole::Rear,   Fen::Punched, 3.2, 1.0, 0, 1.1, 2.4, false},
          {WallRole::Side,   Fen::Punched, 3.4, 0.9, 0, 1.1, 2.3, false}},
         0.0},
        {"modern house", "plate glass + wrapped CORNER windows",
         {{WallRole::Street, Fen::Plate, 0, 0, 1.4, 0.45, 2.7, true},
          {WallRole::Side,   Fen::Plate, 0, 0, 2.2, 0.45, 2.7, false},
          {WallRole::Rear,   Fen::Plate, 0, 0, 1.2, 0.45, 2.7, false},
          {WallRole::Party,  Fen::Blank}},
         2.4, 0.45, 2.7},
        {"glass tower", "curtain wall: the wall IS the opening",
         {{WallRole::Street, Fen::Curtain}, {WallRole::Side, Fen::Curtain},
          {WallRole::Rear,   Fen::Curtain}, {WallRole::Party, Fen::Curtain}},
         0.0},
        {"factory", "high clerestory strip, solid below",
         {{WallRole::Street, Fen::Clerestory, 0, 0, 1.0, 3.3, 4.3, true},
          {WallRole::Side,   Fen::Clerestory, 0, 0, 1.0, 3.3, 4.3, false},
          {WallRole::Rear,   Fen::Clerestory, 0, 0, 1.0, 3.3, 4.3, false},
          {WallRole::Party,  Fen::Blank}},
         0.0},
        {"corner shop", "storefront at grade, punched to the side",
         {{WallRole::Street, Fen::Storefront, 0, 0, 0, 0.25, 2.9, false},
          {WallRole::Side,   Fen::Storefront, 0, 0, 0, 0.25, 2.9, false},
          {WallRole::Rear,   Fen::Punched, 3.0, 1.0, 0, 1.2, 2.4, false},
          {WallRole::Party,  Fen::Blank}},
         0.0},
    };

    const Poly2 plan{Vec2(0, 0), Vec2(13, 0), Vec2(13, 9), Vec2(0, 9)};
    const std::vector<WallRole> roles{WallRole::Street, WallRole::Side,
                                      WallRole::Rear, WallRole::Party};
    const int n = static_cast<int>(sizeof kRecipes / sizeof kRecipes[0]);
    const Real cellW = 350, cellH = 470;
    Svg s;
    svgOpen(s, (std::string(dir) + "/recipes.svg").c_str(), cellW * n + 60, cellH + 170,
            "Lot Lab — one plan, one code path, five recipes");
    s.textPx(28, 68, "Identical geometry and identical code. Everything that differs between "
                     "these five buildings lives in a fenestration TABLE — strategy per wall "
                     "role, plus numbers. Below each plan: the street elevation, where the "
                     "storey's sill/head half of the fact becomes visible.", 12, "#6b7280");

    for (int i = 0; i < n; ++i) {
        const FenRecipe& rec = kRecipes[i];
        const std::vector<Opening> ops = fenestrate(plan, roles, rec);
        const Real bx = 40 + i * cellW + 26, by = 132;
        s.scale = 17.0;
        s.ox = bx + 8;
        s.oy = by;
        drawBlueprint(s, plan, 0.32, ops);

        elevation(s, bx + 8, by + 9 * s.scale + 132, 13.0, 4.6, ops, 0, 17.0,
                  rec.rules[0].fen == Fen::Curtain);

        int doors = 0, wins = 0;
        for (const Opening& o : ops) (o.door ? doors : wins)++;
        char buf[192];
        snprintf(buf, sizeof buf, "%s", rec.name);
        s.textPx(bx, by + 9 * s.scale + 180, buf, 14, "#1d2430", "start", "600");
        snprintf(buf, sizeof buf, "%s", rec.blurb);
        s.textPx(bx, by + 9 * s.scale + 197, buf, 10.5, "#6b7280");
        snprintf(buf, sizeof buf, "%d openings (%d door)  ·  %s", wins + doors, doors,
                 rec.cornerWrap > 0 ? "corner glazing on" : "no corner glazing");
        s.textPx(bx, by + 9 * s.scale + 212, buf, 10, "#9aa1ad");
    }
    (void)seed;
    svgClose(s);
}

static void sheetBlueprint(const char* dir, uint32_t seed) {
    struct Cell { const char* t; const char* d; Poly2 plan; std::vector<Opening> ops; Real th; };
    std::vector<Cell> cells;
    const Real T = 0.34;                       // wall thickness (m)

    auto roles = [](const Poly2& p, const std::vector<WallRole>& r, Real bay,
                    int doorEdge, int doorBay) {
        std::vector<Opening> out;
        for (std::size_t i = 0; i < p.size(); ++i) {
            const Real len = (p[(i + 1) % p.size()] - p[i]).length();
            const auto o = wallProgram(i, len, r[i], bay,
                                       static_cast<int>(i) == doorEdge ? doorBay : -1);
            out.insert(out.end(), o.begin(), o.end());
        }
        return out;
    };

    {   // A house: street front with the door, windows all round, blank party side
        Poly2 p{Vec2(0, 0), Vec2(12, 0), Vec2(12, 9), Vec2(0, 9)};
        std::vector<WallRole> r{WallRole::Street, WallRole::Side, WallRole::Rear,
                                WallRole::Side};
        cells.push_back({"house", "door on the street bay; rear sills raised", p,
                         roles(p, r, 3.0, 0, 1), T});
    }
    {   // A rowhouse unit: party walls BLANK by construction, not by luck
        Poly2 p{Vec2(0, 0), Vec2(6.4, 0), Vec2(6.4, 11), Vec2(0, 11)};
        std::vector<WallRole> r{WallRole::Street, WallRole::Party, WallRole::Rear,
                                WallRole::Party};
        cells.push_back({"rowhouse unit", "party walls carry no openings at all", p,
                         roles(p, r, 2.9, 0, 0), T});
    }
    {   // An L-plan: the wall program follows the plan round every corner
        Poly2 p{Vec2(0, 0), Vec2(15, 0), Vec2(15, 7), Vec2(8, 7), Vec2(8, 13), Vec2(0, 13)};
        std::vector<WallRole> r{WallRole::Street, WallRole::Side, WallRole::Rear,
                                WallRole::Side, WallRole::Rear, WallRole::Side};
        cells.push_back({"L-plan", "one program, six walls, no special cases", p,
                         roles(p, r, 3.2, 0, 2), T});
    }
    {   // A tower floor plate — thicker core walls, curtain bays all round
        Poly2 p = tessellate(filletAllCorners(
            arcFromPoly(Poly2{Vec2(0, 0), Vec2(20, 0), Vec2(20, 18), Vec2(0, 18)}), 2.6), 0.08);
        std::vector<WallRole> r(p.size(), WallRole::Street);
        std::vector<Opening> ops;
        for (std::size_t i = 0; i < p.size(); ++i) {
            const Real len = (p[(i + 1) % p.size()] - p[i]).length();
            if (len < 1.6) continue;               // skip the corner chords
            const auto o = wallProgram(i, len, r[i], 2.2, -1);
            ops.insert(ops.end(), o.begin(), o.end());
        }
        cells.push_back({"tower plate", "rounded corners; bays skip the chord joints",
                         p, ops, 0.28});
    }

    const Real cellW = 360, cellH = 400;
    const int cols = 4;
    Svg s;
    svgOpen(s, (std::string(dir) + "/blueprint.svg").c_str(), cellW * cols + 60,
            cellH + 190, "Lot Lab — the plan as a blueprint");
    s.textPx(28, 68, "Walls have thickness; openings are punched through the poche by boolean. "
                     "An opening is a SPAN ALONG A WALL (plan) plus a sill/head (storey) — "
                     "together they fully determine the 3-D opening, so the blueprint decides "
                     "and the 3-D pass reads.", 12, "#6b7280");

    const Real drawW = cellW - 80, drawH = 250;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const Real bx = 40 + static_cast<Real>(i) * cellW + 26, by = 128;
        Vec2 lo, hi;
        engine::bounds(cells[i].plan, lo, hi);
        const Real w = hi.x - lo.x, h = hi.y - lo.y;
        s.scale = std::min(drawW / std::max(w, Real(1)), drawH / std::max(h, Real(1)));
        s.ox = bx + (drawW - w * s.scale) * 0.5 - lo.x * s.scale;
        s.oy = by + (drawH - h * s.scale) * 0.5 - lo.y * s.scale;
        drawBlueprint(s, cells[i].plan, cells[i].th, cells[i].ops);

        int doors = 0, wins = 0;
        for (const Opening& o : cells[i].ops) (o.door ? doors : wins)++;
        char buf[192];
        snprintf(buf, sizeof buf, "%s", cells[i].t);
        s.textPx(bx, by + drawH + 44, buf, 14, "#1d2430", "start", "600");
        snprintf(buf, sizeof buf, "%s", cells[i].d);
        s.textPx(bx, by + drawH + 61, buf, 11, "#6b7280");
        snprintf(buf, sizeof buf, "%.0f m2  ·  %d doors, %d windows  ·  %.0f cm walls",
                 engine::area(cells[i].plan), doors, wins, cells[i].th * 100);
        s.textPx(bx, by + drawH + 76, buf, 10, "#9aa1ad");
    }
    (void)seed;
    svgClose(s);
}

// ---------------------------------------------------------------------------
// Sheet 7 — THE STACK. Blueprints going UP: base, shaft, crown; several towers
// off one podium; and lofting between plans so a tower is not a prism.
// ---------------------------------------------------------------------------

// LOFT by interpolating the two plans' distance FIELDS, not their vertices.
// Vertex-lerp needs a correspondence and dies the moment the plans differ in
// count or topology; field-lerp handles a square becoming a circle, and one
// loop splitting into two, with no special case. Both halves already exist.
static std::vector<Poly2> loftPlans(const std::vector<Poly2>& A,
                                    const std::vector<Poly2>& B, Real t,
                                    Real twistRad = 0, Real cell = 0.16) {
    Vec2 lo(1e30, 1e30), hi(-1e30, -1e30);
    for (const auto* S : {&A, &B})
        for (const Poly2& p : *S) {
            Vec2 a, b;
            engine::bounds(p, a, b);
            lo = Vec2(std::min(lo.x, a.x), std::min(lo.y, a.y));
            hi = Vec2(std::max(hi.x, b.x), std::max(hi.y, b.y));
        }
    lo = lo - Vec2(3, 3);
    hi = hi + Vec2(3, 3);
    Field f = fieldMake(lo, hi, cell);
    const Vec2 c((lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5);
    auto dist = [](const std::vector<Poly2>& S, const Vec2& q) {
        Real d = 1e30;
        for (const Poly2& p : S) d = std::min(d, sdfPolyAt(p, q));
        return d;
    };
    for (int j = 0; j < f.ny; ++j)
        for (int i = 0; i < f.nx; ++i) {
            const Vec2 q = f.pos(i, j);
            f.d[static_cast<std::size_t>(j) * f.nx + i] =
                dist(A, q) * (1 - t) + dist(B, q) * t;
        }
    std::vector<Poly2> out = orientByNesting(fieldContour(f));
    // TWIST is applied to the finished storey plan, not to the sampling: the
    // plan is what rotates as the tower rises. Rotating one input's frame
    // instead blends an unrotated field with a rotated one, and the levels
    // cross through each other.
    if (std::fabs(twistRad) > 1e-9) {
        const Real cs = std::cos(twistRad), sn = std::sin(twistRad);
        for (Poly2& p : out)
            for (Vec2& v : p) {
                const Vec2 r = v - c;
                v = Vec2(c.x + r.x * cs - r.y * sn, c.y + r.x * sn + r.y * cs);
            }
    }
    return out;
}

static void sheetStack(const char* dir, uint32_t seed) {
    Svg s;
    const Real cellW = 420, cellH = 400;
    const int cols = 3, rows = 2;
    svgOpen(s, (std::string(dir) + "/stack.svg").c_str(), cellW * cols + 60,
            cellH * rows + 180, "Lot Lab — blueprints going up");
    s.textPx(28, 68, "Every tier is CLIPPED to the tier below, so containment is a guarantee, "
                     "not a check. Lofts interpolate distance fields, which survives a change "
                     "of vertex count and even of topology.", 12, "#6b7280");

    const Real drawW = cellW - 90, drawH = 250;
    auto place = [&](int i, Real spanX, Real spanY) {
        const int cx = i % cols, cy = i / cols;
        const Real bx = 40 + cx * cellW + 34, by = 118 + cy * cellH;
        s.scale = std::min(drawW / spanX, drawH / spanY);
        s.ox = bx + drawW * 0.5;
        s.oy = by + drawH * 0.5;
        return std::make_pair(bx, by);
    };
    auto label = [&](Real bx, Real by, const char* t, const char* d, const char* stat) {
        s.textPx(bx, by + drawH + 44, t, 14, "#1d2430", "start", "600");
        s.textPx(bx, by + drawH + 61, d, 11, "#6b7280");
        s.textPx(bx, by + drawH + 76, stat, 10, "#9aa1ad");
    };

    // --- 1: base / podium / shaft / crown, each clipped to the one below ----
    {
        auto [bx, by] = place(0, 46, 46);
        auto roundedSquare = [](Real half, Real r) {
            return tessellate(filletAllCorners(arcFromPoly(Poly2{
                Vec2(-half, -half), Vec2(half, -half),
                Vec2(half, half), Vec2(-half, half)}), r), 0.08);
        };
        std::vector<Poly2> podium{roundedSquare(19, 3.0)};
        // Each tier is AUTHORED on its own terms, then clipped to the one
        // below — that clip is what makes containment a guarantee.
        std::vector<Poly2> shaft = polyBool({roundedSquare(14, 4.5)}, podium, BOp::Intersect);
        std::vector<Poly2> crown =
            polyBool({regularPoly(Vec2(0, 0), 9.0, 32, 0)}, shaft, BOp::Intersect);
        const char* fills[3] = {"#d6dae2", "#c2c8d3", "#aeb6c4"};
        int k = 0;
        for (const auto* L : {&podium, &shaft, &crown}) {
            for (const Poly2& p : *L) s.poly(p, fills[k], "#232a36", 1.6);
            ++k;
        }
        label(bx, by, "1 · base → shaft → crown",
              "each tier clipped to the one below",
              "3 tiers · containment guaranteed by the clip");
    }
    // --- 2: several towers off one podium ---------------------------------
    {
        auto [bx, by] = place(1, 46, 46);
        Poly2 podium{Vec2(-21, -13), Vec2(21, -13), Vec2(21, 13), Vec2(-21, 13)};
        podium = tessellate(filletAllCorners(arcFromPoly(podium), 3.0), 0.1);
        std::vector<Poly2> towers{
            regularPoly(Vec2(-12, 0), 7.4, 6, kTau * 0.08),
            regularPoly(Vec2(7, 2), 6.0, 24, 0),
            Poly2{Vec2(12, -10), Vec2(19, -10), Vec2(19, -3), Vec2(12, -3)}};
        std::vector<Poly2> clipped;
        for (const Poly2& t : towers) {
            auto c = polyBool({t}, {podium}, BOp::Intersect);
            clipped.insert(clipped.end(), c.begin(), c.end());
        }
        s.poly(podium, "#d6dae2", "#232a36", 1.6);
        for (const Poly2& p : clipped) s.poly(p, "#aeb6c4", "#232a36", 1.6);
        label(bx, by, "2 · towers on a podium",
              "one tier holds SEVERAL plans, each clipped",
              "1 podium + 3 towers · different heights, one base");
    }
    // --- 3: loft square -> circle ------------------------------------------
    {
        auto [bx, by] = place(2, 40, 40);
        std::vector<Poly2> A{Poly2{Vec2(-15, -15), Vec2(15, -15), Vec2(15, 15), Vec2(-15, 15)}};
        std::vector<Poly2> B{regularPoly(Vec2(0, 0), 9, 40, 0)};
        for (int k = 0; k <= 6; ++k) {
            const Real t = k / 6.0;
            char col[16];
            snprintf(col, sizeof col, "#%02x%02x%02x", 35 + k * 12, 42 + k * 12, 54 + k * 12);
            for (const Poly2& p : loftPlans(A, B, t))
                s.poly(p, "none", col, k == 0 || k == 6 ? 2.0 : 1.0);
        }
        label(bx, by, "3 · loft: square → circle",
              "7 storey plans, field-interpolated",
              "vertex counts differ — a vertex lerp cannot do this");
    }
    // --- 4: lofted AND twisted ---------------------------------------------
    {
        auto [bx, by] = place(3, 44, 44);
        std::vector<Poly2> A{Poly2{Vec2(-16, -11), Vec2(16, -11), Vec2(16, 11), Vec2(-16, 11)}};
        std::vector<Poly2> B{Poly2{Vec2(-9, -7), Vec2(9, -7), Vec2(9, 7), Vec2(-9, 7)}};
        for (int k = 0; k <= 7; ++k) {
            const Real t = k / 7.0;
            char col[16];
            snprintf(col, sizeof col, "#%02x%02x%02x", 35 + k * 11, 42 + k * 11, 54 + k * 11);
            for (const Poly2& p : loftPlans(A, B, t, t * kTau * 0.10))
                s.poly(p, "none", col, k == 0 || k == 7 ? 2.0 : 1.0);
        }
        label(bx, by, "4 · loft + twist",
              "the upper plan sampled in a rotating frame",
              "8 storey plans · taper and spiral from one op");
    }
    // --- 5: the terrace is a boolean ---------------------------------------
    {
        auto [bx, by] = place(4, 46, 46);
        Poly2 lower{Vec2(-20, -14), Vec2(20, -14), Vec2(20, 14), Vec2(-20, 14)};
        std::vector<Poly2> upper{Poly2{Vec2(-20, -14), Vec2(4, -14), Vec2(4, 8), Vec2(-20, 8)}};
        auto terrace = polyBool({lower}, upper, BOp::Subtract);
        s.poly(lower, "#e6e9ee", "#8a8578", 1.0, "stroke-dasharray=\"5 4\"");
        for (const Poly2& p : terrace) s.poly(p, "#a9c69a", "#4d6b43", 1.6);
        for (const Poly2& p : upper) s.poly(p, "#aeb6c4", "#232a36", 1.8);
        label(bx, by, "5 · terraces fall out",
              "exposed roof = lower plan MINUS upper plan",
              "no special case — the setback terrace is a subtract");
    }
    // --- 6: topology change through the loft --------------------------------
    {
        auto [bx, by] = place(5, 46, 46);
        std::vector<Poly2> A{Poly2{Vec2(-19, -7), Vec2(19, -7), Vec2(19, 7), Vec2(-19, 7)}};
        std::vector<Poly2> B{regularPoly(Vec2(-11, 0), 5.4, 26, 0),
                             regularPoly(Vec2(11, 0), 5.4, 26, 0)};
        for (int k = 0; k <= 5; ++k) {
            const Real t = k / 5.0;
            char col[16];
            snprintf(col, sizeof col, "#%02x%02x%02x", 35 + k * 14, 42 + k * 14, 54 + k * 14);
            for (const Poly2& p : loftPlans(A, B, t))
                s.poly(p, "none", col, k == 0 || k == 5 ? 2.0 : 1.0);
        }
        label(bx, by, "6 · one slab → two towers",
              "the loft SPLITS; the field does not care",
              "topology change for free — the payoff for field lofting");
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
    lab::sheetEngine(out, seed);
    lab::sheetLots(out, seed);
    lab::sheetPlans(out, seed);
    lab::sheetShapes(out, seed);
    lab::sheetCurves(out, seed);
    lab::sheetCompose(out, seed);
    lab::sheetBlueprint(out, seed);
    lab::sheetStack(out, seed);
    lab::sheetRecipes(out, seed);
    lab::sheetCorner(out, seed);
    lab::sheetSilhouette(out, seed);
    lab::sheetPort(out, seed);
    lab::sheetBuildable(out, seed);
    printf("lot_lab: wrote %s/{lots,plans,shapes,curves,compose,blueprint,stack,"
           "engine,recipes,corner,silhouette,port,buildable}.svg "
           "(seed %u)\n", out, seed);
    return 0;
}
