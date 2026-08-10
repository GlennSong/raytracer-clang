#include "mass_stack.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace {

constexpr Real kTau = 6.28318530717958647692;

Real totalArea(const std::vector<Shape2>& v) {
    Real a = 0;
    for (const Shape2& s : v) a += area(s);
    return a;
}

Vec2 sharedCentroid(const std::vector<Shape2>& plans) {
    Real total = 0;
    Vec2 acc;
    for (const Shape2& s : plans) {
        const Real a = area(s);
        if (a <= 0) continue;
        acc = acc + centroid(s) * a;
        total += a;
    }
    return total > 1e-9 ? acc / total : Vec2();
}

}  // namespace

// ---------------------------------------------------------------------------
// Tier / stack basics
// ---------------------------------------------------------------------------

int Tier::levelCount() const {
    int n = 0;
    for (const Storey& s : storeys) n += std::max(0, s.count);
    return n;
}

Real Tier::height() const {
    Real h = 0;
    for (const Storey& s : storeys) h += s.height * std::max(0, s.count);
    return h;
}

Real MassStack::height() const {
    Real h = 0;
    for (const Tier& t : tiers) h += t.height();
    return h;
}

int MassStack::levelCount() const {
    int n = 0;
    for (const Tier& t : tiers) n += t.levelCount();
    return n;
}

// ---------------------------------------------------------------------------
// The rules
// ---------------------------------------------------------------------------

Real profileAt(ProfileKind kind, Real t, Real profileTo) {
    t = std::max(Real(0), std::min(Real(1), t));
    switch (kind) {
        case ProfileKind::Constant: return 1.0;
        case ProfileKind::Taper:    return 1.0 + (profileTo - 1.0) * t;
        case ProfileKind::Swell:
            // Bulge low then nose in — the Gherkin's silhouette as a single
            // scalar curve, which is why it needs no twist and no loft.
            return std::pow(std::sin(kTau * 0.5 * (0.20 + 0.78 * t)), Real(0.85));
        case ProfileKind::Waist:
            return 1.0 - 0.28 * std::sin(kTau * 0.5 * t);
    }
    return 1.0;
}

Real supportRatio(const std::vector<Shape2>& above,
                  const std::vector<Shape2>& below) {
    const Real aboveArea = totalArea(above);
    if (aboveArea <= 1e-9) return 1.0;
    if (below.empty()) return 0.0;
    const std::vector<Shape2> common = shapeBool(above, below, BoolOp::Intersect);
    return std::max(Real(0), std::min(Real(1), totalArea(common) / aboveArea));
}

Real overhangDistance(const std::vector<Shape2>& above,
                      const std::vector<Shape2>& below) {
    if (below.empty()) return 1e9;
    // The part of `above` that is NOT over `below`, and how far it reaches.
    const std::vector<Shape2> out = shapeBool(above, below, BoolOp::Subtract);
    if (out.empty()) return 0.0;
    std::vector<Poly2> bnd;
    for (const Shape2& s : below) {
        bnd.push_back(tessellate(s.outer, 0.1));
        for (const Loop2& h : s.holes) bnd.push_back(tessellate(h, 0.1));
    }
    Real worst = 0;
    for (const Shape2& s : out) {
        for (const Vec2& p : tessellate(s.outer, 0.2)) {
            Real best = 1e30;
            for (const Poly2& l : bnd)
                for (std::size_t i = 0; i < l.size(); ++i) {
                    const Vec2 a = l[i], b = l[(i + 1) % l.size()];
                    const Vec2 d = b - a;
                    const Real len2 = d.lengthSquared();
                    Real u = len2 > 1e-12 ? dot(p - a, d) / len2 : 0;
                    u = std::max(Real(0), std::min(Real(1), u));
                    best = std::min(best, (p - (a + d * u)).length());
                }
            worst = std::max(worst, best);
        }
    }
    return worst;
}

std::vector<Shape2> enforceSupport(const std::vector<Shape2>& above,
                                   const std::vector<Shape2>& below,
                                   const SupportRule& rule) {
    if (below.empty() || above.empty()) return above;
    const Real ratio = supportRatio(above, below);
    const Real over = rule.maxOverhang > 0 ? overhangDistance(above, below) : 0;
    if (ratio >= rule.minSupport - 1e-6 && over <= rule.maxOverhang + 1e-6)
        return above;
    // The allowed envelope is `below` grown by the permitted overhang — with
    // the default rule that is `below` itself, i.e. plain containment.
    std::vector<Shape2> allowed;
    if (rule.maxOverhang > 1e-6) {
        for (const Shape2& s : below) {
            std::vector<Shape2> g = offsetShape(s, rule.maxOverhang);
            allowed.insert(allowed.end(), g.begin(), g.end());
        }
        if (allowed.empty()) allowed = below;
    } else {
        allowed = below;
    }
    std::vector<Shape2> clipped = shapeBool(above, allowed, BoolOp::Intersect);
    return clipped.empty() ? below : clipped;
}

std::vector<Shape2> loftPlans(const std::vector<Shape2>& a,
                              const std::vector<Shape2>& b, Real t, Real cell) {
    if (b.empty() || t <= 1e-6) return a;
    if (a.empty() || t >= 1 - 1e-6) return b;
    // Blend the two SIGNED DISTANCE FIELDS and contour the result. A
    // vertex-to-vertex interpolation cannot change topology, so it could never
    // merge two towers into one arch or open a plate into a donut; the field
    // does both for free, which is exactly why the plan keeps it around.
    std::vector<Shape2> all = a;
    all.insert(all.end(), b.begin(), b.end());
    Field2 fa = rasterize(a, cell, cell * 6);
    Field2 fb = rasterize(b, cell, cell * 6);
    Field2 f = rasterize(all, cell, cell * 6);
    if (f.nx < 2) return a;
    for (int y = 0; y < f.ny; ++y)
        for (int x = 0; x < f.nx; ++x) {
            const Vec2 p(f.lo.x + x * f.cell, f.lo.y + y * f.cell);
            const Real da = fa.nx >= 2 ? fa.sample(p) : Real(1e30);
            const Real db = fb.nx >= 2 ? fb.sample(p) : Real(1e30);
            f.d[static_cast<std::size_t>(y) * f.nx + x] = da * (1 - t) + db * t;
        }
    std::vector<Poly2> loops = contour(f, 0.0);
    std::vector<Loop2> ls;
    for (const Poly2& p : loops)
        if (p.size() >= 3) ls.push_back(loopFromPoly(p));
    std::vector<Shape2> out = assembleShapes(std::move(ls));
    return out.empty() ? a : out;
}

std::vector<Shape2> transformPlans(const std::vector<Shape2>& plans, Real scale,
                                   Real twist) {
    if (plans.empty()) return plans;
    const Vec2 c = sharedCentroid(plans);
    const Real s = std::max(Real(0.02), scale);
    const Real cs = std::cos(twist), sn = std::sin(twist);
    auto xf = [&](Loop2& l) {
        for (Edge2& e : l.edges) {
            const Vec2 r = (e.a - c) * s;
            e.a = Vec2(c.x + r.x * cs - r.y * sn, c.y + r.x * sn + r.y * cs);
        }
        // Bulge is scale- and rotation-invariant (it is tan of a quarter of the
        // included ANGLE), so a scaled arc stays the same arc, correctly.
    };
    std::vector<Shape2> out = plans;
    for (Shape2& sh : out) {
        xf(sh.outer);
        for (Loop2& h : sh.holes) xf(h);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Expansion
// ---------------------------------------------------------------------------

std::vector<Level> expandLevels(const MassStack& stack) {
    std::vector<Level> out;
    Real y = stack.base.topY;
    std::vector<Shape2> below;      // the level immediately underneath
    for (std::size_t ti = 0; ti < stack.tiers.size(); ++ti) {
        const Tier& tier = stack.tiers[ti];
        const int n = tier.levelCount();
        if (n <= 0 || tier.plans.empty()) continue;
        int idx = 0;
        for (const Storey& band : tier.storeys) {
            for (int k = 0; k < std::max(0, band.count); ++k, ++idx) {
                // t runs across the WHOLE tier, so a profile curve or a loft
                // spans the tier rather than restarting at each storey band.
                const Real t = n > 1 ? static_cast<Real>(idx) / (n - 1) : 0;
                std::vector<Shape2> plans =
                    tier.loftTo.empty() ? tier.plans
                                        : loftPlans(tier.plans, tier.loftTo, t);
                plans = transformPlans(plans,
                                       profileAt(tier.profile, t, tier.profileTo),
                                       tier.twistPerLevel * idx);
                // Support is a TIER-TO-TIER constraint, checked where one tier
                // lands on the next — not between every pair of levels. Within
                // a tier the profile curve and the loft ARE the design: a
                // Gherkin's bulge is an overhang by definition, and clipping
                // each level to the one below would flatten it back into a
                // prism. (The plan says exactly this: a tier's plan is produced
                // independently, THEN constrained against the tier below.)
                if (idx == 0 && !below.empty())
                    plans = enforceSupport(plans, below, tier.support);
                if (plans.empty()) continue;
                Level lv;
                lv.plans = plans;
                lv.y0 = y;
                lv.y1 = y + band.height;
                lv.tier = static_cast<int>(ti);
                lv.indexInTier = idx;
                lv.materialSet = tier.materialSet;
                y = lv.y1;
                below = lv.plans;
                out.push_back(std::move(lv));
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Convenience stacks
// ---------------------------------------------------------------------------

namespace {

Foundation plinthFor(const Shape2& plan) {
    Foundation f;
    std::vector<Shape2> g = offsetShape(plan, 0.35);
    f.plan = g.empty() ? plan : g[0];
    f.topY = 0;
    f.thickness = 0.45;
    return f;
}

}  // namespace

MassStack simpleStack(const Shape2& plan, int floors, Real floorHeight) {
    MassStack s;
    s.base = plinthFor(plan);
    Tier t;
    t.plans = {plan};
    // The ground floor is its own band: taller, and the thing every retail and
    // lobby recipe wants to set independently.
    t.storeys.push_back(Storey{floorHeight * 1.25, 1});
    if (floors > 1) t.storeys.push_back(Storey{floorHeight, floors - 1});
    s.tiers.push_back(std::move(t));
    return s;
}

MassStack podiumTower(const Shape2& podium, const Shape2& tower, int podiumFloors,
                      int towerFloors, Real floorHeight) {
    MassStack s;
    s.base = plinthFor(podium);
    Tier p;
    p.plans = {podium};
    p.storeys.push_back(Storey{floorHeight * 1.3, 1});
    if (podiumFloors > 1) p.storeys.push_back(Storey{floorHeight, podiumFloors - 1});
    p.materialSet = 0;
    s.tiers.push_back(std::move(p));
    Tier t;
    t.plans = {tower};
    t.storeys.push_back(Storey{floorHeight, std::max(1, towerFloors)});
    t.materialSet = 1;                 // the shaft re-clads at the transition
    s.tiers.push_back(std::move(t));
    return s;
}

MassStack weddingCake(const Shape2& plan, int tiers, int floorsPerTier,
                      Real shrinkPerTier, Real floorHeight) {
    MassStack s;
    s.base = plinthFor(plan);
    Shape2 cur = plan;
    for (int i = 0; i < std::max(1, tiers); ++i) {
        Tier t;
        t.plans = {cur};
        t.storeys.push_back(Storey{floorHeight, std::max(1, floorsPerTier)});
        t.materialSet = i == 0 ? 0 : 1;
        s.tiers.push_back(std::move(t));
        Vec2 lo, hi;
        bounds(cur, lo, hi);
        const Real step = std::min(hi.x - lo.x, hi.y - lo.y) * shrinkPerTier * 0.5;
        std::vector<Shape2> next = offsetShape(cur, -std::max(Real(0.5), step));
        if (next.empty()) break;
        cur = next[0];
    }
    return s;
}

MassStack gherkin(const Shape2& plan, int floors, Real floorHeight) {
    MassStack s;
    s.base = plinthFor(plan);
    Tier t;
    t.plans = {plan};
    t.storeys.push_back(Storey{floorHeight, std::max(2, floors)});
    // ONE plan, a profile curve, NO loft and NO twist. The silhouette comes
    // entirely from how big the floor is at each height.
    t.profile = ProfileKind::Swell;
    s.tiers.push_back(std::move(t));
    return s;
}

MassStack pyramid(const Shape2& plan, int floors, Real floorHeight) {
    MassStack s;
    s.base = plinthFor(plan);
    Tier t;
    t.plans = {plan};
    t.storeys.push_back(Storey{floorHeight, std::max(2, floors)});
    t.profile = ProfileKind::Taper;
    t.profileTo = 0.06;              // very nearly a point
    s.tiers.push_back(std::move(t));
    return s;
}

MassStack archTower(const Shape2& legA, const Shape2& legB, int floors,
                    Real floorHeight) {
    MassStack s;
    s.base = plinthFor(legA);
    // The CROWN is a single mass spanning both legs — the beam across the top.
    // Lofting toward the legs' plain UNION would be a no-op, because two
    // disjoint shapes united are still two disjoint shapes; the target has to
    // be the thing that actually bridges them.
    Vec2 loA, hiA, loB, hiB;
    bounds(legA, loA, hiA);
    bounds(legB, loB, hiB);
    const Shape2 crown = rectShape(std::min(loA.x, loB.x), std::min(loA.y, loB.y),
                                   std::max(hiA.x, hiB.x), std::max(hiA.y, hiB.y));
    Tier t;
    t.plans = {legA, legB};
    t.loftTo = {crown};
    t.storeys.push_back(Storey{floorHeight, std::max(3, floors)});
    // The blended field closes the gap partway up: below that the contour comes
    // back as two loops, above it as one. That merge IS the arch, and only the
    // sampled field can produce it — no vertex morph changes topology.
    t.support.minSupport = 0.35;
    t.support.maxOverhang = 1e9;
    s.tiers.push_back(std::move(t));
    return s;
}

}  // namespace engine
