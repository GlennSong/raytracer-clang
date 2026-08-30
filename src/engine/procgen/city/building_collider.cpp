#include "building_collider.h"

#include "road_mesh.h"   // triangulatePolygon
#include <algorithm>

namespace engine {

void appendBuildingPrism(std::vector<Vec3>& V, std::vector<uint32_t>& I,
                         const Poly2& plan, Real base, Real top, Real floorY,
                         const std::vector<DoorSpec>& doors, Real plinth) {
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
