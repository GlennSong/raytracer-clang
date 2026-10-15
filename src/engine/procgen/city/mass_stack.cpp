#include "mass_stack.h"
#include <algorithm>
#include <cmath>

namespace engine {

// ---------------------------------------------------------------------------------------------------------------
// Curves and transforms

Real Curve::at(Real t, Real def) const {
    if (keys.empty()) return def;
    if (t <= keys.front().first) return keys.front().second;
    if (t >= keys.back().first) return keys.back().second;
    for (std::size_t i = 1; i < keys.size(); ++i) {
        if (t > keys[i].first) continue;
        const Real t0 = keys[i - 1].first, t1 = keys[i].first;
        const Real f = t1 > t0 ? (t - t0) / (t1 - t0) : 0;
        return keys[i - 1].second + (keys[i].second - keys[i - 1].second) * f;
    }
    return keys.back().second;
}

bool Curve::flat(Real v) const {
    for (const auto& k : keys)
        if (k.second != v) return false;
    return true;
}

Curve Curve::constant(Real v) { return Curve{{{0, v}}}; }
Curve Curve::linear(Real v0, Real v1) { return Curve{{{0, v0}, {1, v1}}}; }

bool Transform::identity() const {
    return rot.flat(0) && scaleU.flat(1) && scaleV.flat(1) && slideU.flat(0) && slideV.flat(0);
}

Section Transform::apply(const Section& s, Real t) const {
    if (identity()) return s;
    const Real a = rot.at(t, 0) * (3.14159265358979323846 / 180.0);
    const Real sU = scaleU.at(t, 1), sV = scaleV.at(t, 1);
    const Real dU = slideU.at(t, 0), dV = slideV.at(t, 0);
    const Real ca = std::cos(a), sa = std::sin(a);
    auto rotate = [&](const Vec2& p) { return Vec2(p.x * ca - p.y * sa, p.x * sa + p.y * ca); };
    // p' = pivot + slide + A (p - pivot), A = sU R(u) u^T + sV R(v) v^T: a linear map, handed to transformed()
    // as the images of the plan's x and y axes.
    const Vec2 ru = rotate(u) * sU, rv = rotate(v) * sV;
    const Vec2 col0 = ru * u.x + rv * v.x;   // A (1, 0)
    const Vec2 col1 = ru * u.y + rv * v.y;   // A (0, 1)
    const Vec2 Ap = col0 * pivot.x + col1 * pivot.y;
    const Vec2 origin = pivot + u * dU + v * dV - Ap;
    return s.transformed(origin, col0, col1);
}

// ---------------------------------------------------------------------------------------------------------------
// The stack

int Stack::entryOf(int k) const {
    int e = -1;
    for (std::size_t i = 0; i < entries.size(); ++i)
        if (entries[i].k0 <= k) e = static_cast<int>(i);
    return e;
}

Section Stack::at(int k) const {
    const int e = entryOf(k);
    if (e < 0) return {};
    const StackEntry& en = entries[static_cast<std::size_t>(e)];
    const Real span = static_cast<Real>(std::max(1, en.k1 - en.k0));
    const Real t = std::max(Real(0), std::min(Real(1), static_cast<Real>(k - en.k0) / span));
    return en.xf.apply(en.section, t);
}

std::vector<MassTier> Stack::tiers() const {
    std::vector<MassTier> out;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const StackEntry& en = entries[i];
        const int floor0 = i == 0 ? 0 : en.k0 - 1;
        if (!en.varies()) { out.push_back({en.section.tessellate(), floor0}); continue; }
        // a transformed entry: constant over its storeys, one tier; keyed, one per storey (the core's fit asks
        // every storey it climbs through)
        bool keyed = false;
        for (const Curve* c : {&en.xf.rot, &en.xf.scaleU, &en.xf.scaleV, &en.xf.slideU, &en.xf.slideV})
            for (const auto& key : c->keys)
                if (key.second != c->keys.front().second) keyed = true;
        if (!keyed) { out.push_back({at(en.k0).tessellate(), floor0}); continue; }
        for (int k = en.k0; k <= std::min(en.k1, floors); ++k)
            out.push_back({at(k).tessellate(), i == 0 && k == 0 ? 0 : k - 1});
    }
    return out;
}

Slice Stack::slice(Real y) const {
    Slice s;
    if (entries.empty()) return s;
    int k = 0;
    if (y >= groundH) k = 1 + static_cast<int>(std::floor((y - groundH) / std::max(Real(0.1), floorH) + 1e-6));
    k = std::max(0, std::min(floors, k));
    const int e = entryOf(k);
    if (e < 0) return s;
    s.pieces.push_back(at(k));
    s.tier = e;
    return s;
}

MassBounds Stack::bounds() const {
    MassBounds b;
    bool first = true;
    for (int k = 0; k <= floors; ++k) {
        for (const Vec2& p : at(k).tessellate()) {
            if (first) { b.lo = b.hi = p; first = false; continue; }
            b.lo.x = std::min(b.lo.x, p.x); b.lo.y = std::min(b.lo.y, p.y);
            b.hi.x = std::max(b.hi.x, p.x); b.hi.y = std::max(b.hi.y, p.y);
        }
    }
    b.y0 = 0;
    b.y1 = height();
    return b;
}

RenderMesh Stack::shell(int lod, const Vec3& colour) const {
    (void)lod;   // a stack's native form IS the loft
    RenderMesh m;
    std::vector<Poly2> outlines;
    std::vector<Real> heights;
    for (int k = 0; k <= floors; ++k) {
        outlines.push_back(at(k).tessellate());
        heights.push_back(k == 0 ? 0.0 : groundH + (k - 1) * floorH);
    }
    outlines.push_back(at(floors).tessellate());
    heights.push_back(height());
    loftSlices(m, outlines, heights, colour, true);
    return m;
}

// ---------------------------------------------------------------------------------------------------------------
// The preset

namespace {

// Every storey's outline of `entry` inside `plan` (each corner nudged 2% to the plan's centre, as the tiers test).
bool entryInside(const Poly2& plan, const StackEntry& entry, const Vec2& centre) {
    for (int k = entry.k0; k <= entry.k1; ++k) {
        const Real span = static_cast<Real>(std::max(1, entry.k1 - entry.k0));
        const Real t = static_cast<Real>(k - entry.k0) / span;
        for (const Vec2& q : entry.xf.apply(entry.section, t).tessellate())
            if (!pointInPolygon(plan, q + (centre - q) * 0.02)) return false;
    }
    return true;
}

Section scaledAbout(const Poly2& plan, const Vec2& c, Real s) {
    Poly2 p;
    for (const Vec2& q : plan) p.push_back(c + (q - c) * s);
    return Section::fromLot(p);
}

}  // namespace

Stack stackOf(const Poly2& planIn, const BuildingParams& params) {
    Stack st;
    st.groundH = params.groundHeight;
    st.floorH = params.floorHeight;
    st.floors = params.floors;
    Poly2 plan = planIn;
    if (plan.size() < 3) return st;
    ensureCCW(plan);
    using Env = BuildingParams::Envelope;
    if (params.envelope == Env::Twist || params.envelope == Env::Stack) {
        // THE BASE: the lot plan up to `baseFloors` (storey f0 included), as the taper's.
        const int f0 = std::max(1, params.baseFloors);
        StackEntry base;
        base.section = Section::fromLot(plan);
        base.k0 = 0;
        base.k1 = f0 >= params.floors ? params.floors : f0;
        st.entries.push_back(base);
        if (f0 >= params.floors) return st;
        const OBB2 ob = orientedBoundingBox(plan);
        if (params.envelope == Env::Twist) {
            // THE TWIST: one entry from f0 + 1 to the top, the plan itself (or the box inside it) turning
            // `twistDeg` about its centre over the height, scaled down until every storey stays inside the lot.
            StackEntry tw;
            tw.k0 = f0 + 1;
            tw.k1 = params.floors;
            tw.rel = Relation::Support;
            tw.xf.pivot = ob.center;
            tw.xf.u = ob.axis[0];
            tw.xf.v = ob.axis[1];
            tw.xf.rot = Curve::linear(0, params.twistDeg);
            bool placed = false;
            for (int pass = 0; pass < 2 && !placed; ++pass) {
                Poly2 shape = plan;
                if (pass == 1) {   // the box inside the plan (an L or a notched lot cannot turn inside itself)
                    Real hw = 0, hd = 0;
                    if (!tierBoxInside(plan, ob, hw, hd)) break;
                    shape = tierBox(ob, hw, hd);
                }
                for (Real s = 1.0; s >= 0.58; s -= 0.04) {
                    tw.section = scaledAbout(shape, ob.center, s);
                    if (tierShortSide(tw.section.tessellate()) < kCoreTierMin * 0.8) break;
                    if (entryInside(plan, tw, ob.center)) { placed = true; break; }
                }
            }
            if (placed) st.entries.push_back(tw);
            return st;
        }
        // THE STACK: modules of `stepFloors` storeys, the box inside the plan shrunk along the long axis by the
        // shift, slid `stackShift` alternately (0, +d, 0, -d, ...) so every module stays inside the lot and each
        // hangs past the one below by the shift at most.
        Real hw = 0, hd = 0;
        if (!tierBoxInside(plan, ob, hw, hd)) return st;
        const int la = ob.longAxis();
        const Real d = std::max(Real(0), params.stackShift);
        Real halfL = (la == 0 ? hw : hd) - d, halfS = la == 0 ? hd : hw;
        if (2 * halfL < kCoreTierMin) { halfL = (la == 0 ? hw : hd); }   // too tight to shift inside: no shift
        const Real shift = (la == 0 ? hw : hd) - halfL;
        const Poly2 box = la == 0 ? tierBox(ob, halfL, halfS) : tierBox(ob, halfS, halfL);
        const int m = std::max(2, params.stepFloors);
        int j = 0;
        for (int k = f0 + 1; k <= params.floors; k += m, ++j) {
            StackEntry mod;
            mod.section = Section::fromLot(box);
            mod.k0 = k;
            mod.k1 = std::min(params.floors, k + m - 1);
            mod.rel = Relation::Support;
            mod.maxOverhang = shift;
            mod.xf.pivot = ob.center;
            mod.xf.u = ob.axis[la];
            mod.xf.v = ob.axis[1 - la];
            const Real s = (j % 2 == 1) ? shift * ((j / 2) % 2 == 0 ? 1.0 : -1.0) : 0.0;
            if (s != 0) mod.xf.slideU = Curve::constant(s);
            st.entries.push_back(mod);
        }
        return st;
    }
    // TODAY'S TIERS as entries with identity transforms: byte-identical storeys.
    const std::vector<MassTier> tiers = envelopeTiers(plan, params);
    for (std::size_t t = 0; t < tiers.size(); ++t) {
        StackEntry e;
        e.section = Section::fromLot(tiers[t].plan);
        e.k0 = t == 0 ? 0 : tiers[t].floor0 + 1;
        e.k1 = t + 1 < tiers.size() ? tiers[t + 1].floor0 : params.floors;
        e.rel = Relation::Nest;
        st.entries.push_back(e);
    }
    return st;
}

}  // namespace engine
