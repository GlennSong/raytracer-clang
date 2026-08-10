#include "plan_grammar.h"

#include "rng.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

constexpr Real kTau = 6.28318530717958647692;

// The largest region of a boolean result — the answer when an op was meant to
// reshape one mass and the algebra handed back several.
Shape2 largest(const std::vector<Shape2>& v) {
    if (v.empty()) return Shape2{};
    std::size_t best = 0;
    for (std::size_t i = 1; i < v.size(); ++i)
        if (area(v[i]) > area(v[best])) best = i;
    return v[best];
}

// A local frame on edge `i`: its start, direction along it, and its OUTWARD
// normal. Every op that grows or bites is expressed in this frame, which is why
// they all work at any angle rather than only on axis-aligned walls.
struct EdgeFrame {
    Vec2 a, b, dir, outward;
    Real length = 0;
};

EdgeFrame frameOf(const Loop2& loop, std::size_t i) {
    EdgeFrame f;
    f.a = loop.start(i);
    f.b = loop.end(i);
    const Vec2 d = f.b - f.a;
    f.length = d.length();
    f.dir = f.length > 1e-9 ? d / f.length : Vec2(1, 0);
    f.outward = Vec2(f.dir.y, -f.dir.x);      // right of a CCW edge
    return f;
}

// The rectangle standing on a span of an edge, `depth` deep along the outward
// normal (negative bites inward). This one helper is outset, wing, court and
// notch — they differ only in the span and the sign.
Shape2 slabOn(const EdgeFrame& f, Real t0, Real t1, Real depth) {
    const Vec2 p0 = f.a + f.dir * (f.length * t0);
    const Vec2 p1 = f.a + f.dir * (f.length * t1);
    Poly2 ring = {p0, p1, p1 + f.outward * depth, p0 + f.outward * depth};
    Shape2 s = shapeFromPoly(ring);
    return s;
}

// The span [t0,t1] of an edge that a limb of `width` occupies, centred at
// `along`. A zero or oversized width means the whole edge.
void spanFor(Real edgeLen, Real width, Real along, Real& t0, Real& t1) {
    if (width <= 0 || width >= edgeLen) { t0 = 0; t1 = 1; return; }
    const Real halfT = (width / edgeLen) * 0.5;
    Real c = std::max(halfT, std::min(1 - halfT, along));
    t0 = c - halfT;
    t1 = c + halfT;
}

}  // namespace

// ---------------------------------------------------------------------------
// Selectors
// ---------------------------------------------------------------------------

EdgeSelector EdgeSelector::all() { return EdgeSelector{}; }
EdgeSelector EdgeSelector::byTag(EdgeTag t) {
    EdgeSelector s; s.by = By::Tag; s.tag = t; return s;
}
EdgeSelector EdgeSelector::longest(int n) {
    EdgeSelector s; s.by = By::Longest; s.count = n; return s;
}
EdgeSelector EdgeSelector::shortest(int n) {
    EdgeSelector s; s.by = By::Shortest; s.count = n; return s;
}
EdgeSelector EdgeSelector::facing(const Vec2& d, Real deg) {
    EdgeSelector s; s.by = By::Facing; s.dir = d; s.degrees = deg; return s;
}
EdgeSelector EdgeSelector::longerThan(Real len) {
    EdgeSelector s; s.by = By::LongerThan; s.length = len; return s;
}
EdgeSelector EdgeSelector::at(std::vector<int> idx) {
    EdgeSelector s; s.by = By::Index; s.indices = std::move(idx); return s;
}

std::vector<std::size_t> EdgeSelector::select(const Loop2& loop) const {
    const std::size_t n = loop.size();
    std::vector<std::size_t> out;
    if (n < 3) return out;
    switch (by) {
        case By::All:
            for (std::size_t i = 0; i < n; ++i) out.push_back(i);
            break;
        case By::Tag:
            for (std::size_t i = 0; i < n; ++i)
                if (loop.edges[i].tag == tag) out.push_back(i);
            break;
        case By::Index:
            for (int i : indices)
                if (i >= 0 && static_cast<std::size_t>(i) < n)
                    out.push_back(static_cast<std::size_t>(i));
            break;
        case By::Longest:
        case By::Shortest: {
            std::vector<std::pair<Real, std::size_t>> len;
            for (std::size_t i = 0; i < n; ++i)
                len.push_back({(loop.end(i) - loop.start(i)).length(), i});
            std::sort(len.begin(), len.end(),
                      [&](const auto& a, const auto& b) {
                          return by == By::Longest ? a.first > b.first
                                                   : a.first < b.first;
                      });
            for (int k = 0; k < count && k < static_cast<int>(len.size()); ++k)
                out.push_back(len[k].second);
            std::sort(out.begin(), out.end());
            break;
        }
        case By::Facing: {
            const Vec2 want = normalize(dir);
            const Real lim = std::cos(std::max(Real(0), degrees) * kTau / 360.0);
            for (std::size_t i = 0; i < n; ++i) {
                const EdgeFrame f = frameOf(loop, i);
                if (f.length > 1e-9 && dot(f.outward, want) >= lim)
                    out.push_back(i);
            }
            break;
        }
        case By::LongerThan:
            for (std::size_t i = 0; i < n; ++i)
                if ((loop.end(i) - loop.start(i)).length() >= length)
                    out.push_back(i);
            break;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Plan quality
// ---------------------------------------------------------------------------

bool PlanQuality::ok(const Shape2& shape, std::string* why) const {
    auto fail = [&](const char* m) {
        if (why) *why = m;
        return false;
    };
    if (shape.outer.size() < 3) return fail("degenerate outline");
    if (area(shape) < minArea) return fail("below minimum area");

    const Loop2& l = shape.outer;
    const std::size_t n = l.size();
    for (std::size_t i = 0; i < n; ++i) {
        // Arc edges are measured by ARC LENGTH: the chord of a strongly bowed
        // wall can be short while the wall itself is long, and rejecting that
        // would forbid apses and bow fronts outright.
        const ArcGeom g = arcGeom(l.start(i), l.end(i), l.bulge(i));
        const Real len = g.straight ? (l.end(i) - l.start(i)).length()
                                    : std::fabs(g.radius * g.sweep);
        if (len < minEdge) return fail("wall shorter than minEdge");
    }
    const Real minCos = std::cos(minAngleDeg * kTau / 360.0);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        // Tangent directions AT the corner, so a wall meeting an arc is
        // measured against the arc's tangent rather than its chord.
        const Vec2 pIn = arcPoint(l.start(prev), l.end(prev), l.bulge(prev), 0.98);
        const Vec2 pOut = arcPoint(l.start(i), l.end(i), l.bulge(i), 0.02);
        const Vec2 din = normalize(l.edges[i].a - pIn);
        const Vec2 dout = normalize(pOut - l.edges[i].a);
        if (din.lengthSquared() < 0.5 || dout.lengthSquared() < 0.5) continue;
        // Interior angle = pi - turn. A knife edge is a tiny interior angle,
        // i.e. the outgoing direction nearly reverses the incoming one.
        const Real c = dot(din, dout);
        if (-c > minCos) return fail("interior angle below minAngle");
    }
    // A notch narrower than a room: any two NON-ADJACENT walls closer than
    // minNotch pinch the plan into something unusable.
    const Poly2 t = tessellate(shape.outer, 0.15);
    const std::size_t m = t.size();
    for (std::size_t i = 0; i < m && m > 5; ++i) {
        for (std::size_t j = i + 2; j < m; ++j) {
            if (i == 0 && j == m - 1) continue;
            const Real gap = (t[i] - t[j]).length();
            if (gap >= minNotch) continue;
            // Only a real pinch counts: points far apart ALONG the boundary but
            // close in space. Neighbours around a rounded corner are not that.
            const std::size_t walk = std::min(j - i, m - (j - i));
            if (walk > 3) return fail("notch narrower than a room");
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Ops
// ---------------------------------------------------------------------------

Shape2 applyPlanOp(const Shape2& shape, const PlanOp& op) {
    if (shape.outer.size() < 3 || op.kind == PlanOpKind::Noop) return shape;
    const std::vector<std::size_t> sel = op.on.select(shape.outer);
    if (sel.empty()) return shape;

    switch (op.kind) {
        case PlanOpKind::Noop:
            return shape;

        case PlanOpKind::Fillet:
        case PlanOpKind::Chamfer: {
            std::vector<Real> r(shape.outer.size(), 0.0);
            for (std::size_t i : sel) r[i] = op.depth;
            Shape2 out = shape;
            out.outer = op.kind == PlanOpKind::Fillet
                            ? filletCorners(shape.outer, r)
                            : chamferCorners(shape.outer, r);
            return out;
        }

        case PlanOpKind::Bow: {
            Shape2 out = shape;
            for (std::size_t i : sel) bowEdge(out.outer, i, op.depth);
            return out;
        }

        case PlanOpKind::Outset:
        case PlanOpKind::Wing: {
            // Both ADD material standing on an edge; a wing is simply an outset
            // over part of the edge rather than all of it.
            std::vector<Shape2> add;
            for (std::size_t i : sel) {
                const EdgeFrame f = frameOf(shape.outer, i);
                if (f.length < 1e-6) continue;
                Real t0, t1;
                spanFor(f.length, op.kind == PlanOpKind::Wing ? op.width : 0,
                        op.along, t0, t1);
                add.push_back(slabOn(f, t0, t1, std::fabs(op.depth)));
            }
            if (add.empty()) return shape;
            std::vector<Shape2> acc{shape};
            for (const Shape2& s : add) acc = shapeBool(acc, {s}, BoolOp::Unite);
            return largest(acc);
        }

        case PlanOpKind::Inset:
        case PlanOpKind::Court:
        case PlanOpKind::Notch: {
            // All three REMOVE material: the cut stands on the edge and reaches
            // inward. Court and Notch take a span; Inset takes the whole wall.
            std::vector<Shape2> cut;
            for (std::size_t i : sel) {
                const EdgeFrame f = frameOf(shape.outer, i);
                if (f.length < 1e-6) continue;
                Real t0, t1;
                const Real w = op.kind == PlanOpKind::Inset ? 0 : op.width;
                spanFor(f.length, w, op.along, t0, t1);
                // Reach slightly PAST the wall so the cut cannot leave a
                // hairline sliver of material behind the face it bites.
                Shape2 s = slabOn(f, t0, t1, -std::fabs(op.depth));
                Shape2 lip = slabOn(f, t0, t1, 0.01);
                std::vector<Shape2> u = shapeBool({s}, {lip}, BoolOp::Unite);
                cut.push_back(u.empty() ? s : largest(u));
            }
            if (cut.empty()) return shape;
            std::vector<Shape2> acc{shape};
            for (const Shape2& s : cut)
                acc = shapeBool(acc, {s}, BoolOp::Subtract);
            return largest(acc);
        }
    }
    return shape;
}

Shape2 clipToEnvelope(const Shape2& shape, const Shape2& envelope) {
    if (envelope.outer.size() < 3) return shape;
    std::vector<Shape2> r = shapeBool({shape}, {envelope}, BoolOp::Intersect);
    return largest(r);
}

Shape2 runPlanScript(const Shape2& seed, const PlanScript& script,
                     std::uint32_t rngSeed, const PlanQuality& quality,
                     const Shape2* envelope) {
    Shape2 cur = seed;
    if (envelope) cur = clipToEnvelope(cur, *envelope);
    Rng rng(rngSeed);
    for (const PlanStep& step : script.steps) {
        if (step.choices.empty()) continue;
        // Weighted choice INCLUDING whatever Noop the author put in the list.
        Real total = 0;
        for (const PlanOp& o : step.choices) total += std::max(Real(0), o.weight);
        if (total <= 0) continue;
        Real r = rng.unit() * total;
        const PlanOp* pick = &step.choices.back();
        for (const PlanOp& o : step.choices) {
            r -= std::max(Real(0), o.weight);
            if (r <= 0) { pick = &o; break; }
        }
        Shape2 next = applyPlanOp(cur, *pick);
        if (envelope) next = clipToEnvelope(next, *envelope);
        // The invariant is checked HERE, after every op — and a failing op is
        // reverted rather than the whole plan being thrown away. A plan is
        // never allowed to exist in a state the stack would amplify.
        if (next.outer.size() >= 3 && quality.ok(next)) cur = next;
    }
    return cur;
}

// ---------------------------------------------------------------------------
// Named templates
// ---------------------------------------------------------------------------

const char* planTemplateName(PlanTemplate t) {
    switch (t) {
        case PlanTemplate::Bar:              return "bar";
        case PlanTemplate::L:                return "L";
        case PlanTemplate::U:                return "U";
        case PlanTemplate::H:                return "H";
        case PlanTemplate::T:                return "T";
        case PlanTemplate::Courtyard:        return "courtyard";
        case PlanTemplate::Flatiron:         return "flatiron";
        case PlanTemplate::Cruciform:        return "cruciform";
        case PlanTemplate::ChamferedSquare:  return "chamfered-square";
        case PlanTemplate::CurvedCornerSlab: return "curved-corner-slab";
        case PlanTemplate::Pinwheel:         return "pinwheel";
        case PlanTemplate::Octagon:          return "octagon";
        case PlanTemplate::BowFront:         return "bow-front";
        case PlanTemplate::Pentagon:         return "pentagon";
        case PlanTemplate::Hexagon:          return "hexagon";
        case PlanTemplate::RadialWings:      return "radial-wings";
        case PlanTemplate::Trefoil:          return "trefoil";
        case PlanTemplate::CrossApse:        return "cross-apse";
        case PlanTemplate::AtriumRing:       return "atrium-ring";
        case PlanTemplate::LobedTower:       return "lobed-tower";
        case PlanTemplate::WedgeTower:       return "wedge-tower";
        case PlanTemplate::SawtoothShed:     return "sawtooth-shed";
        case PlanTemplate::CourtyardWings:   return "courtyard-wings";
    }
    return "?";
}

std::vector<PlanTemplate> allPlanTemplates() {
    return {PlanTemplate::Bar,       PlanTemplate::L,
            PlanTemplate::U,         PlanTemplate::H,
            PlanTemplate::T,         PlanTemplate::Courtyard,
            PlanTemplate::Flatiron,  PlanTemplate::Cruciform,
            PlanTemplate::ChamferedSquare, PlanTemplate::CurvedCornerSlab,
            PlanTemplate::Pinwheel,  PlanTemplate::Octagon,
            PlanTemplate::BowFront,  PlanTemplate::Pentagon,
            PlanTemplate::Hexagon,   PlanTemplate::RadialWings,
            PlanTemplate::Trefoil,   PlanTemplate::CrossApse,
            PlanTemplate::AtriumRing, PlanTemplate::LobedTower,
            PlanTemplate::WedgeTower, PlanTemplate::SawtoothShed,
            PlanTemplate::CourtyardWings};
}

Shape2 regularPolygon(int sides, Real w, Real d, Real rotation) {
    sides = std::max(3, sides);
    // Inscribed in the frame, so an n-gon fills the same box a rectangle would
    // and every composed plan below can be sized the same way.
    const Vec2 c(w * 0.5, d * 0.5);
    Poly2 ring;
    for (int i = 0; i < sides; ++i) {
        const Real a = rotation + kTau * (static_cast<Real>(i) / sides);
        ring.push_back(Vec2(c.x + std::cos(a) * w * 0.5,
                            c.y + std::sin(a) * d * 0.5));
    }
    return shapeFromPoly(ring);
}

namespace {

// Blend a disc onto a shape with a soft join — the brief's "SDF join circles to
// the ends of the wings". Only the sampled field can do this: a fillet between
// two masses is not expressible in loop algebra.
Shape2 blendDisc(const Shape2& base, const Vec2& at, Real r, Real k) {
    // Fine enough that the contour, not the sampling, is what the fit sees;
    // coarse enough that a 60 m plate does not rasterize a million cells.
    const Real cell = std::max(Real(0.15), std::min(Real(0.45), r * 0.05));
    std::vector<Shape2> b = smoothUnion(base, circleShape(at, r), k, cell);
    if (b.empty()) return base;
    // Straight out of the field a blend is hundreds of half-metre chords. Refit
    // it as the arcs it actually is, at the sampling's own accuracy — otherwise
    // every wall is shorter than a door and the mass stack multiplies the
    // vertex count by the storey count.
    return fitArcs(largest(b), cell * 0.9, 3.0);
}

// The outward normal at the midpoint of edge i, and that midpoint.
void edgeMid(const Loop2& l, std::size_t i, Vec2& mid, Vec2& outward) {
    const EdgeFrame f = frameOf(l, i);
    mid = f.a + f.dir * (f.length * 0.5);
    outward = f.outward;
}

// The shortest wall a composed template will deliberately build. Sits above
// PlanQuality::minEdge so a template that fits a feature by this measure still
// passes the invariant after the boolean's rounding.
constexpr Real kMinWall = 2.8;

// Chamfer every corner by the smallest cut that leaves a BUILDABLE diagonal.
// A fixed fraction of the frame cannot do this: the diagonal a cut produces is
// 2t*sin(theta/2), so a 60-degree corner needs nearly twice the cut a right
// angle does, and on a small frame there may be no cut that works at all — in
// which case that corner stays sharp, which is the honest answer rather than a
// 1-metre stub wall.
Shape2 chamferBuildable(const Shape2& s, Real want, Real minWall) {
    const Loop2& l = s.outer;
    const std::size_t n = l.size();
    std::vector<Real> cuts(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        if (std::fabs(l.bulge(i)) > 1e-9 || std::fabs(l.bulge(prev)) > 1e-9)
            continue;
        const Vec2 P = l.edges[prev].a, V = l.edges[i].a, N = l.end(i);
        const Vec2 din = normalize(V - P), dout = normalize(N - V);
        if (din.lengthSquared() < 0.5 || dout.lengthSquared() < 0.5) continue;
        const Real interior = kTau * 0.5 - std::acos(std::max(
            Real(-1), std::min(Real(1), dot(din, dout))));
        const Real half = std::sin(interior * 0.5);
        if (half < 1e-3) continue;
        const Real need = minWall / (2 * half);
        const Real cut = std::max(want, need);
        // Both walls the cut eats into must survive it, and they are eaten from
        // BOTH ends, so each corner may only spend half of what is spare.
        const Real spare = (std::min((V - P).length(), (N - V).length()) - minWall) * 0.5;
        if (cut <= spare) cuts[i] = cut;
    }
    Shape2 out = s;
    out.outer = chamferCorners(l, cuts);
    return out;
}

}  // namespace

Shape2 planTemplate(PlanTemplate t, Real w, Real d, std::uint32_t seed) {
    w = std::max(w, Real(4));
    d = std::max(d, Real(4));
    Rng rng(seed * 2654435761u + 1u);
    const Shape2 frame = rectShape(0, 0, w, d);

    switch (t) {
        case PlanTemplate::Bar:
            return frame;

        case PlanTemplate::L: {
            // Bite one corner out. The bite is a fraction of each side, so the
            // arms stay proportional at any frame size.
            const Real bw = w * rng.range(0.38, 0.52);
            const Real bd = d * rng.range(0.38, 0.52);
            return largest(subtract(frame, rectShape(w - bw, d - bd, w + 1, d + 1)));
        }

        case PlanTemplate::U: {
            const Real slot = w * rng.range(0.30, 0.42);
            const Real deep = d * rng.range(0.55, 0.72);
            const Real x0 = (w - slot) * 0.5;
            return largest(subtract(frame, rectShape(x0, d - deep, x0 + slot, d + 1)));
        }

        case PlanTemplate::H: {
            const Real slot = w * rng.range(0.26, 0.36);
            const Real deep = d * rng.range(0.28, 0.38);
            const Real x0 = (w - slot) * 0.5;
            std::vector<Shape2> a = subtract(frame, rectShape(x0, d - deep, x0 + slot, d + 1));
            if (a.empty()) return frame;
            return largest(subtract(a[0], rectShape(x0, -1, x0 + slot, deep)));
        }

        case PlanTemplate::T: {
            const Real stem = w * rng.range(0.30, 0.42);
            const Real head = d * rng.range(0.30, 0.42);
            const Real x0 = (w - stem) * 0.5;
            std::vector<Shape2> a = subtract(frame, rectShape(-1, -1, x0, d - head));
            if (a.empty()) return frame;
            return largest(subtract(a[0], rectShape(x0 + stem, -1, w + 1, d - head)));
        }

        case PlanTemplate::Courtyard: {
            // A real hole, not a U — the thing a bare point ring cannot express.
            const Real bx = w * rng.range(0.22, 0.30);
            const Real by = d * rng.range(0.22, 0.30);
            return largest(subtract(frame, rectShape(bx, by, w - bx, d - by)));
        }

        case PlanTemplate::Flatiron: {
            // The wedge at a street fork. Built with a BLUNT NOSE from the
            // start rather than as a knife edge that is chamfered afterwards:
            // a real flatiron has a narrow end wall (the Flatiron Building's
            // prow is about 6 m across), and chamfering a point leaves stub
            // walls far shorter than a wall is allowed to be.
            const Real nose = std::max(Real(3.5), d * rng.range(0.16, 0.26));
            const Real cy = d * 0.5;
            Poly2 ring = {{0, cy + nose * 0.5},
                          {0, cy - nose * 0.5},
                          {w, 0},
                          {w, d}};
            return shapeFromPoly(ring);
        }

        case PlanTemplate::Cruciform: {
            const Real ax = w * rng.range(0.28, 0.36);
            const Real ay = d * rng.range(0.28, 0.36);
            std::vector<Shape2> acc{frame};
            const Real corners[4][4] = {{-1, -1, ax, ay},
                                        {w - ax, -1, w + 1, ay},
                                        {-1, d - ay, ax, d + 1},
                                        {w - ax, d - ay, w + 1, d + 1}};
            for (const auto& c : corners) {
                acc = shapeBool(acc, {rectShape(c[0], c[1], c[2], c[3])},
                                BoolOp::Subtract);
                if (acc.empty()) return frame;
            }
            return largest(acc);
        }

        case PlanTemplate::ChamferedSquare: {
            // The corner-entrance cut: one corner taken off at 45 degrees, wide
            // enough for a door to sit in the diagonal.
            std::vector<Real> cuts(4, 0.0);
            cuts[0] = std::min(w, d) * rng.range(0.18, 0.28);
            Shape2 s = frame;
            s.outer = chamferCorners(frame.outer, cuts);
            return s;
        }

        case PlanTemplate::CurvedCornerSlab: {
            std::vector<Real> radii(4, 0.0);
            radii[0] = std::min(w, d) * rng.range(0.28, 0.45);
            Shape2 s = frame;
            s.outer = filletCorners(frame.outer, radii);
            return s;
        }

        case PlanTemplate::Pinwheel: {
            // Two opposite corners bitten, leaving arms that rotate about the
            // centre. The bites are capped so the WAIST between them stays a
            // room wide — at full fraction on a small frame the two bites very
            // nearly meet and the plan pinches to nothing, which the quality
            // invariant rightly rejects. Clamping here is the template taking
            // responsibility for being valid at any size it is asked for.
            const Real waist = 3.5;
            const Real ax = std::min(w * rng.range(0.34, 0.44), (w - waist) * 0.5);
            const Real ay = std::min(d * rng.range(0.34, 0.44), (d - waist) * 0.5);
            if (ax <= 0.5 || ay <= 0.5) return frame;
            std::vector<Shape2> acc{frame};
            acc = shapeBool(acc, {rectShape(-1, -1, ax, ay)}, BoolOp::Subtract);
            if (acc.empty()) return frame;
            acc = shapeBool(acc, {rectShape(w - ax, d - ay, w + 1, d + 1)},
                            BoolOp::Subtract);
            if (acc.empty()) return frame;
            return largest(acc);
        }

        case PlanTemplate::Octagon: {
            Shape2 s = frame;
            s.outer = chamferCorners(frame.outer,
                                     std::vector<Real>(4, std::min(w, d) * 0.29));
            return s;
        }

        case PlanTemplate::Pentagon:
            return regularPolygon(5, w, d, kTau * 0.25);
        case PlanTemplate::Hexagon:
            return regularPolygon(6, w, d, 0);

        case PlanTemplate::RadialWings: {
            // THE BRIEF, literally: start from a polygon, grow a wing off every
            // face, then blend a disc onto each wing's end. Composed entirely
            // from grammar ops and shape_ops — no new geometry code.
            const int sides = rng.irange(5, 6);
            // The core is kept small and the wings long: a radial plan reads
            // as ARMS off a hub, and a fat hub with stubby wings is a starfish.
            // Core plus reach is held inside the frame, since the envelope the
            // recipe fitted this to is a real boundary.
            Shape2 core = regularPolygon(sides, w * 0.44, d * 0.44,
                                         sides == 5 ? kTau * 0.25 : 0);
            // Centre it in the frame.
            Vec2 clo, chi;
            bounds(core, clo, chi);
            const Vec2 shift(w * 0.5 - (clo.x + chi.x) * 0.5,
                             d * 0.5 - (clo.y + chi.y) * 0.5);
            for (Edge2& e : core.outer.edges) e.a = e.a + shift;

            // A wing costs its own width plus a THROAT on either side of it —
            // the gap between neighbouring wings is what the plan invariant
            // measures as a notch. The core edge length is therefore the budget,
            // and when it cannot pay, the wings are dropped rather than pinched:
            // the discs then blend straight onto the polygon's corners, which is
            // the same idea at the size the frame can actually hold.
            Real coreEdge = 1e30;
            for (std::size_t i = 0; i < core.outer.size(); ++i)
                coreEdge = std::min(coreEdge, frameOf(core.outer, i).length);
            const Real throat = 3.2;
            const Real reach = std::max(Real(3.0), std::min(w, d) * rng.range(0.23, 0.27));
            const Real wingW = std::min(std::min(w, d) * 0.20, coreEdge - throat);
            // A wing is a LIMB: rooms either side of a corridor, six metres at
            // the very least. Narrower than that and the plan is not a winged
            // building, it is a knobbly blob whose every feature is smaller
            // than a room — which is also exactly when the invariant starts
            // rejecting it. A radial-winged plan is a large-site plan.
            const Real kMinWing = 6.0;

            Shape2 s = core;
            std::vector<Vec2> tips;
            if (wingW >= kMinWing) {
                PlanOp wing;
                wing.kind = PlanOpKind::Wing;
                wing.on = EdgeSelector::all();
                wing.depth = reach;
                wing.width = wingW;
                s = applyPlanOp(core, wing);
                for (std::size_t i = 0; i < core.outer.size(); ++i) {
                    Vec2 mid, out;
                    edgeMid(core.outer, i, mid, out);
                    // The cap sits ON the wing's end face, so its circle is
                    // TANGENT to both side walls: an apsidal end. A wider disc
                    // pushed further out overhangs the wing and re-enters the
                    // side walls at a grazing angle — a cusp, and a cusp is the
                    // one thing the plan invariant will not accept.
                    tips.push_back(mid + out * reach);
                }
            } else {
                for (const Edge2& e : core.outer.edges) tips.push_back(e.a);
            }

            // A disc blended onto the tip of every wing. A tangent cap needs
            // almost no blend — the join is already smooth, and a big k would
            // only scoop a short concave arc out of either side of it. The
            // wingless fallback DOES need one, because there the discs cut
            // across the core at a real angle.
            const bool winged = wingW >= kMinWing;
            const Real r = std::max(Real(2.0),
                                    winged ? wingW * 0.5 : coreEdge * 0.30);
            for (const Vec2& tip : tips)
                s = blendDisc(s, tip, r, r * (winged ? 0.25 : 0.9));
            return s;
        }

        case PlanTemplate::Trefoil: {
            // Three lobes smooth-joined into one organic plate.
            // The lobes have to be far enough apart to READ as three and close
            // enough to still be one building: centres a distance off*sqrt(3)
            // apart, which has to stay under 2R for the discs to overlap at all.
            const Vec2 c(w * 0.5, d * 0.5);
            const Real R = std::min(w, d) * 0.27;
            const Real off = std::min(w, d) * 0.25;
            Shape2 s = circleShape(Vec2(c.x, c.y + off), R);
            for (int i = 1; i < 3; ++i) {
                const Real a = kTau * 0.25 + kTau * (static_cast<Real>(i) / 3);
                s = blendDisc(s, Vec2(c.x + std::cos(a) * off,
                                      c.y + std::sin(a) * off), R, R * 0.45);
            }
            return s;
        }

        case PlanTemplate::CrossApse: {
            // A cruciform with a true semicircular apse on one arm — a church
            // plan, and the reason arcs had to survive booleans.
            Shape2 s = planTemplate(PlanTemplate::Cruciform, w, d, seed);
            if (s.outer.size() < 4) return s;
            // Apse the northernmost wall.
            std::size_t best = 0;
            Real bestY = -1e30;
            for (std::size_t i = 0; i < s.outer.size(); ++i) {
                const Vec2 m = (s.outer.start(i) + s.outer.end(i)) * 0.5;
                const Vec2 dd = s.outer.end(i) - s.outer.start(i);
                if (std::fabs(dd.y) > std::fabs(dd.x)) continue;   // want a
                if (m.y > bestY) { bestY = m.y; best = i; }        // horizontal
            }
            const Real chord = (s.outer.end(best) - s.outer.start(best)).length();
            bowEdge(s.outer, best, chord * rng.range(0.34, 0.48));
            return s;
        }

        case PlanTemplate::AtriumRing: {
            // A ring with a REAL hole, its corners rounded and a notch cut for
            // the entrance — three ops on one seed. Each op is spent only if the
            // ring can afford it: the wall between the street and the atrium is
            // the budget, and a fillet or a notch that eats through it turns a
            // building into a broken one.
            Shape2 s = planTemplate(PlanTemplate::Courtyard, w, d, seed);
            if (s.holes.empty()) return s;
            Vec2 hlo, hhi, olo, ohi;
            Shape2 holeOnly;
            holeOnly.outer = s.holes[0];
            bounds(holeOnly, hlo, hhi);
            bounds(s, olo, ohi);
            const Real wall = std::min(hlo.x - olo.x, hlo.y - olo.y);
            const Real holeShort = std::min(hhi.x - hlo.x, hhi.y - hlo.y);

            // A fillet's ARC has to be a wall in its own right: an arc of radius
            // r spans r*pi/2, so a small radius is a wall too short to build.
            const Real r = std::max(Real(1.9), std::min(w, d) * 0.09);
            if (holeShort >= 2 * r + kMinWall && std::min(w, d) >= 2 * r + kMinWall)
                s = filletAll(s, r);

            const Real depth = kMinWall;
            if (wall >= depth + kMinWall) {
                PlanOp notch;
                notch.kind = PlanOpKind::Notch;
                notch.on = EdgeSelector::facing(Vec2(0, -1), 35);
                notch.depth = depth;
                notch.width = std::max(Real(3.4), std::min(w, d) * 0.16);
                s = applyPlanOp(s, notch);
            }
            return s;
        }

        case PlanTemplate::LobedTower: {
            // Every face bowed out: a clover-leaf tower plate. One op, four
            // edges, and the arcs stay true circles all the way to the mesher.
            Shape2 s = rectShape(w * 0.16, d * 0.16, w * 0.84, d * 0.84);
            PlanOp bow;
            bow.kind = PlanOpKind::Bow;
            bow.on = EdgeSelector::all();
            bow.depth = std::min(w, d) * rng.range(0.10, 0.15);
            return applyPlanOp(s, bow);
        }

        case PlanTemplate::WedgeTower: {
            // The narrow end has to survive being chamfered from both ends, so
            // it is sized from the cut rather than the cut being sized from it.
            const Real cut = std::max(Real(2.2), std::min(w, d) * 0.10);
            const Real narrow = std::min(w - kMinWall,
                                         std::max(w * rng.range(0.36, 0.52),
                                                  2 * cut + kMinWall));
            Poly2 ring = {{(w - narrow) * 0.5, 0},
                          {(w + narrow) * 0.5, 0},
                          {w, d},
                          {0, d}};
            return chamferBuildable(shapeFromPoly(ring), cut, kMinWall);
        }

        case PlanTemplate::SawtoothShed: {
            // An industrial shed whose long rear edge is notched into bays. The
            // pitch comes from a real bay (a truss spans 15-20 m), and a notch
            // is only cut where a whole one — walls, floor and the pier beside
            // it — fits within that pitch.
            Shape2 s = rectShape(0, 0, w, d);
            const int bays = std::max(2, static_cast<int>(std::lround(w / 16.0)));
            const Real pitch = w / bays;
            const Real bite = std::max(kMinWall, pitch * 0.5);
            const Real depth = std::max(kMinWall, d * 0.20);
            // The END piers are half a pitch wide, so they bind before the piers
            // between notches do — clear those and every pier is buildable.
            if (pitch - bite < 2 * kMinWall || d - depth < kMinWall) return s;
            PlanOp notch;
            notch.kind = PlanOpKind::Notch;
            notch.on = EdgeSelector::facing(Vec2(0, 1), 30);
            notch.depth = depth;
            notch.width = bite;
            for (int i = 0; i < bays; ++i) {
                // Measured against the ORIGINAL edge each time: every notch
                // splits the rear wall, so a fraction of "the selected edge"
                // would land somewhere different on every pass.
                const Real centre = (static_cast<Real>(i) + 0.5) * pitch;
                Shape2 cut = rectShape(centre - bite * 0.5, d - depth,
                                       centre + bite * 0.5, d + 1);
                std::vector<Shape2> r = subtract(s, cut);
                if (!r.empty()) s = largest(r);
            }
            return s;
        }

        case PlanTemplate::CourtyardWings: {
            // The university quad: two wings off a spine, open to a court on one
            // side and closed around a REAL courtyard on the other. Built as
            // subtractions on one frame rather than as an outset on an H — an
            // outset moves every south-facing wall, including the two the quad
            // needs to stay put, and leaves stub walls where they step.
            const Real wall = std::max(kMinWall, std::min(w, d) * 0.16);
            const Real slot = std::max(kMinWall, std::min(w * 0.34, w - 2 * wall));
            const Real x0 = (w - slot) * 0.5;
            const Real link = std::max(kMinWall, d * 0.12);
            Shape2 s = frame;

            // The closed court is spent only if the frame can hold link + court
            // + wall + an open slot; below that the quad is a U, which is what a
            // small enough site actually gets built as.
            const Real court = d - link - wall - std::max(kMinWall, d * 0.24);
            if (court >= kMinWall) {
                std::vector<Shape2> a = subtract(
                    s, rectShape(x0, link, x0 + slot, link + court));
                if (!a.empty()) s = largest(a);
            }
            std::vector<Shape2> b = subtract(
                s, rectShape(x0, d - std::max(kMinWall, d * 0.24), x0 + slot, d + 1));
            return b.empty() ? s : largest(b);
        }

        case PlanTemplate::BowFront: {
            // The bow bulges OUTWARD, so it has to be given room INSIDE the
            // w x d frame — otherwise the finished shape always overhangs the
            // envelope it was sized to and is trimmed straight back off again.
            // Measured before this: every single bow-front was clipped.
            const Real sag = d * rng.range(0.10, 0.16);
            Shape2 s = rectShape(0, sag, w, d);
            bowEdge(s.outer, 0, sag);          // bows down into the reserved band
            return s;
        }
    }
    return frame;
}

}  // namespace engine
