#include "building_collider.h"

#include "road_mesh.h"       // triangulatePolygon
#include "shape_grammar.h"   // colliderTiers
#include "triangulate.h"     // triangulateIndices (a cap with holes)
#include <algorithm>

namespace engine {

void appendBuildingPrism(std::vector<Vec3>& V, std::vector<uint32_t>& I,
                         const Poly2& plan, Real base, Real top, Real floorY,
                         const std::vector<DoorSpec>& doors, Real plinth,
                         const std::vector<Poly2>& roofHoles) {
    if (plan.size() < 3) return;
    const std::size_t n = plan.size();
    auto quad = [&](const Vec2& a, const Vec2& b, Real y0, Real y1) {
        if (y1 - y0 < 1e-4) return;
        const uint32_t s = static_cast<uint32_t>(V.size());
        V.push_back(Vec3(a.x, y0, a.y));
        V.push_back(Vec3(b.x, y0, b.y));
        V.push_back(Vec3(b.x, y1, b.y));
        V.push_back(Vec3(a.x, y1, a.y));
        I.insert(I.end(), {s, s + 1, s + 2, s, s + 2, s + 3});
    };
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = plan[i], b = plan[(i + 1) % n];
        const Vec2 d = b - a;
        const Real len = d.length();
        if (len < 1e-4) continue;
        const Vec2 u = d * (1.0 / len);
        // Doors on this edge: the foot's nearest projection within 0.3 m.
        struct Span { Real t0, t1, head; };
        std::vector<Span> spans;
        for (const DoorSpec& ds : doors) {
            const Real t = dot(ds.foot - a, u);
            if (t < -0.3 || t > len + 0.3) continue;
            const Vec2 q = a + u * std::max(Real(0), std::min(len, t));
            if ((q - ds.foot).length() > 0.3) continue;
            const Real hw = std::max(Real(0.4), ds.width * 0.5);
            spans.push_back({std::max(Real(0), t - hw),
                             std::min(len, t + hw),
                             floorY + std::max(Real(2.0), ds.height)});
        }
        std::sort(spans.begin(), spans.end(),
                  [](const Span& x, const Span& y) { return x.t0 < y.t0; });
        Real cur = 0;
        for (const Span& sp : spans) {
            if (sp.t0 > cur)
                quad(a + u * cur, a + u * sp.t0, base, top);  // wall before it
            quad(a + u * sp.t0, a + u * sp.t1, base, floorY);  // plinth face
            quad(a + u * sp.t0, a + u * sp.t1, sp.head, top);  // lintel + above
            cur = std::max(cur, sp.t1);
        }
        if (cur < len) quad(a + u * cur, a + u * len, base, top);
    }
    // Roof cap (as before) and the FLOOR CAP (the missing collider: the
    // drawn Ground slab had no physics, so anything inside stood on the
    // terrain pad 0.5 m below the floor it could see).
    for (const Real y : {top, floorY}) {
        // The roof cap keeps its holes (the stairs rise through them); the floor cap is whole.
        if (y == top && !roofHoles.empty()) {
            const uint32_t s = static_cast<uint32_t>(V.size());
            for (const Vec2& q : plan) V.push_back(Vec3(q.x, y, q.y));
            for (const Poly2& hole : roofHoles) for (const Vec2& q : hole) V.push_back(Vec3(q.x, y, q.y));
            for (const auto& tri : triangulateIndices(plan, roofHoles))
                I.insert(I.end(), {s + static_cast<uint32_t>(tri[0]), s + static_cast<uint32_t>(tri[1]),
                                   s + static_cast<uint32_t>(tri[2])});
            continue;
        }
        for (const auto& tri : triangulatePolygon(plan)) {
            const uint32_t s = static_cast<uint32_t>(V.size());
            for (int k = 0; k < 3; ++k)
                V.push_back(Vec3(plan[tri[k]].x, y, plan[tri[k]].y));
            I.insert(I.end(), {s, s + 1, s + 2});
        }
    }
    // Threshold step outside each door (tall plinths only).
    if (plinth > 0.3) {
        for (const DoorSpec& ds : doors) {
            const Vec2 out = ds.normal;
            const Vec2 t(-out.y, out.x);
            const Real hw = std::max(Real(0.5), ds.width * 0.5 + 0.1);
            const Real stepY = floorY - plinth * 0.5;
            const Vec2 c = ds.foot;
            const Vec2 corners[4] = {c - t * hw, c + t * hw,
                                     c + t * hw + out * 0.6,
                                     c - t * hw + out * 0.6};
            const uint32_t s = static_cast<uint32_t>(V.size());
            for (const Vec2& p2 : corners)
                V.push_back(Vec3(p2.x, stepY, p2.y));
            I.insert(I.end(), {s, s + 1, s + 2, s, s + 2, s + 3});
            for (int k = 0; k < 4; ++k)  // riser skirt, solid from the side
                quad(corners[k], corners[(k + 1) % 4], stepY - plinth, stepY);
        }
    }
}

bool appendBuildingTiers(std::vector<Vec3>& V, std::vector<uint32_t>& I, const Poly2& plan,
                         const BuildingParams& params, Real baseY, Real groundY,
                         const std::vector<DoorSpec>& doors, const std::vector<Poly2>& deckHoles,
                         const std::vector<Poly2>& roofHoles) {
    const std::vector<ColliderTier> tiers = colliderTiers(plan, params);
    if (tiers.empty() || plan.size() < 3) return false;
    auto quad = [&](const Vec2& a, const Vec2& b, Real y0, Real y1) {
        if (y1 - y0 < 1e-4 || (b - a).lengthSquared() < 1e-8) return;
        const uint32_t s = static_cast<uint32_t>(V.size());
        V.push_back(Vec3(a.x, y0, a.y));
        V.push_back(Vec3(b.x, y0, b.y));
        V.push_back(Vec3(b.x, y1, b.y));
        V.push_back(Vec3(a.x, y1, a.y));
        I.insert(I.end(), {s, s + 1, s + 2, s, s + 2, s + 3});
    };
    for (std::size_t t = 0; t < tiers.size(); ++t) {
        const ColliderTier& ct = tiers[t];
        const bool last = t + 1 == tiers.size();
        const std::vector<Poly2>& holes = last ? roofHoles : deckHoles;
        if (ct.plan.size() < 3) continue;
        if (t == 0) {
            // The ground tier: the prism as before -- door notches, floor cap, threshold steps -- capped at ITS deck.
            appendBuildingPrism(V, I, ct.plan, baseY - 0.5, baseY + ct.y1, baseY + 0.05, doors,
                                baseY - groundY, holes);
        } else {
            // An upper tier: plain walls from the deck below to its own, and its cap (none where the storey above
            // continues the tier on a turned outline: the walls meet the next walls, mass_stack.h).
            for (std::size_t i = 0; i < ct.plan.size(); ++i)
                quad(ct.plan[i], ct.plan[(i + 1) % ct.plan.size()], baseY + ct.y0, baseY + ct.y1);
            if (!ct.deck) continue;
            const uint32_t s = static_cast<uint32_t>(V.size());
            for (const Vec2& q : ct.plan) V.push_back(Vec3(q.x, baseY + ct.y1, q.y));
            for (const Poly2& hole : holes) for (const Vec2& q : hole) V.push_back(Vec3(q.x, baseY + ct.y1, q.y));
            for (const auto& tri : triangulateIndices(ct.plan, holes))
                I.insert(I.end(), {s + static_cast<uint32_t>(tri[0]), s + static_cast<uint32_t>(tri[1]),
                                   s + static_cast<uint32_t>(tri[2])});
        }
        // The parapet on the deck: a ring a walker meets at the edge (mirrorTriangles makes it two-sided) -- in
        // metre pieces, the ones under an overhanging tier above left out (they would wall its room).
        if (ct.parapet > 0)
            for (std::size_t i = 0; i < ct.plan.size(); ++i) {
                const Vec2 a = ct.plan[i], b = ct.plan[(i + 1) % ct.plan.size()];
                if (last) { quad(a, b, baseY + ct.y1, baseY + ct.y1 + ct.parapet); continue; }
                const Poly2& above = tiers[t + 1].plan;
                const int n = std::max(1, static_cast<int>(std::ceil((b - a).length())));
                for (int k = 0; k < n; ++k) {
                    const Vec2 p0 = a + (b - a) * (static_cast<Real>(k) / n), p1 = a + (b - a) * (static_cast<Real>(k + 1) / n);
                    if (above.size() >= 3 && pointInPolygon(above, (p0 + p1) * 0.5)) continue;
                    quad(p0, p1, baseY + ct.y1, baseY + ct.y1 + ct.parapet);
                }
            }
    }
    return true;
}

void appendRoofItems(std::vector<Vec3>& V, std::vector<uint32_t>& I, const std::vector<RoofItem>& items) {
    auto quad = [&](const Vec2& a, const Vec2& b, Real y0, Real y1) {
        if (y1 - y0 < 1e-4 || (b - a).lengthSquared() < 1e-8) return;
        const uint32_t s = static_cast<uint32_t>(V.size());
        V.push_back(Vec3(a.x, y0, a.y));
        V.push_back(Vec3(b.x, y0, b.y));
        V.push_back(Vec3(b.x, y1, b.y));
        V.push_back(Vec3(a.x, y1, a.y));
        I.insert(I.end(), {s, s + 1, s + 2, s, s + 2, s + 3});
    };
    for (const RoofItem& ri : items) {
        if (ri.footprint.size() < 3 || ri.h <= 0) continue;
        const Real y1 = ri.y0 + ri.h;
        if (ri.kind == RoofItem::Kind::Ramp && ri.footprint.size() == 4) {
            // the slope from the foot edge on the deck to the top edge on the pad: the way up
            const Vec2 &A = ri.footprint[0], &B = ri.footprint[1], &Cc = ri.footprint[2], &D = ri.footprint[3];
            const uint32_t s = static_cast<uint32_t>(V.size());
            V.push_back(Vec3(A.x, ri.y0, A.y)); V.push_back(Vec3(B.x, ri.y0, B.y));
            V.push_back(Vec3(Cc.x, y1, Cc.y)); V.push_back(Vec3(D.x, y1, D.y));
            I.insert(I.end(), {s, s + 1, s + 2, s, s + 2, s + 3});
            continue;
        }
        if (ri.kind == RoofItem::Kind::Sloped && ri.footprint.size() == 4) {
            // the glass slope itself, rising from its first edge on the deck to its third edge `h` up, the tall
            // back face under that edge, and the two triangular flanks: the deck beneath stays floor
            const Vec2 &A = ri.footprint[0], &B = ri.footprint[1], &Cc = ri.footprint[2], &D = ri.footprint[3];
            const uint32_t s = static_cast<uint32_t>(V.size());
            V.push_back(Vec3(A.x, ri.y0, A.y)); V.push_back(Vec3(B.x, ri.y0, B.y));
            V.push_back(Vec3(Cc.x, y1, Cc.y)); V.push_back(Vec3(D.x, y1, D.y));
            V.push_back(Vec3(Cc.x, ri.y0, Cc.y)); V.push_back(Vec3(D.x, ri.y0, D.y));
            I.insert(I.end(), {s, s + 1, s + 2, s, s + 2, s + 3});           // the slope
            I.insert(I.end(), {s + 4, s + 5, s + 3, s + 4, s + 3, s + 2});   // the back face
            I.insert(I.end(), {s + 1, s + 4, s + 2});                        // the flanks
            I.insert(I.end(), {s + 5, s, s + 3});
            continue;
        }
        for (std::size_t i = 0; i < ri.footprint.size(); ++i)
            quad(ri.footprint[i], ri.footprint[(i + 1) % ri.footprint.size()], ri.y0, y1);
        if (ri.kind == RoofItem::Kind::Screen || ri.kind == RoofItem::Kind::Frame) continue;   // a ring, open above
        const uint32_t s = static_cast<uint32_t>(V.size());
        for (const Vec2& q : ri.footprint) V.push_back(Vec3(q.x, y1, q.y));
        for (const auto& tri : triangulateIndices(ri.footprint, {}))
            I.insert(I.end(), {s + static_cast<uint32_t>(tri[0]), s + static_cast<uint32_t>(tri[1]), s + static_cast<uint32_t>(tri[2])});
    }
}

void mirrorTriangles(std::vector<uint32_t>& I) {
    const std::size_t oneSided = I.size();
    I.reserve(oneSided * 2);
    for (std::size_t i = 0; i + 2 < oneSided; i += 3) {
        I.push_back(I[i]);
        I.push_back(I[i + 2]);
        I.push_back(I[i + 1]);
    }
}

}  // namespace engine
