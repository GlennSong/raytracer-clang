#include "drawn_road.h"

#include "components.h"
#include "world.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
long long cellKey(int cx, int cz) {
    return static_cast<long long>(cx) * 73856093LL ^ static_cast<long long>(cz) * 19349663LL;
}

bool inTri(const DrawnRoad::Tri& t, double x, double z) {
    const double d1 = (x - t.b.x) * (t.a.y - t.b.y) - (t.a.x - t.b.x) * (z - t.b.y);
    const double d2 = (x - t.c.x) * (t.b.y - t.c.y) - (t.b.x - t.c.x) * (z - t.c.y);
    const double d3 = (x - t.a.x) * (t.c.y - t.a.y) - (t.c.x - t.a.x) * (z - t.a.y);
    const bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
    return !(neg && pos);
}

double segDist(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double l2 = ab.lengthSquared();
    double t = l2 > 1e-12 ? dot(p - a, ab) / l2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    return (p - (a + ab * t)).length();
}
}  // namespace

void DrawnRoad::add(const Vec2& a, const Vec2& b, const Vec2& c) {
    const int i = static_cast<int>(tris.size());
    tris.push_back({a, b, c});
    const double x0 = std::min({a.x, b.x, c.x}), x1 = std::max({a.x, b.x, c.x});
    const double z0 = std::min({a.y, b.y, c.y}), z1 = std::max({a.y, b.y, c.y});
    for (int cx = static_cast<int>(std::floor(x0 / kCell)); cx <= static_cast<int>(std::floor(x1 / kCell)); ++cx)
        for (int cz = static_cast<int>(std::floor(z0 / kCell)); cz <= static_cast<int>(std::floor(z1 / kCell)); ++cz)
            cells[cellKey(cx, cz)].push_back(i);
}

bool DrawnRoad::covers(double x, double z) const {
    auto it = cells.find(cellKey(static_cast<int>(std::floor(x / kCell)), static_cast<int>(std::floor(z / kCell))));
    if (it == cells.end()) return false;
    for (int ti : it->second)
        if (inTri(tris[static_cast<std::size_t>(ti)], x, z)) return true;
    return false;
}

bool DrawnRoad::near(double x, double z, double margin) const {
    if (margin <= 0) return covers(x, z);
    const Vec2 p(x, z);
    const int cx0 = static_cast<int>(std::floor((x - margin) / kCell));
    const int cx1 = static_cast<int>(std::floor((x + margin) / kCell));
    const int cz0 = static_cast<int>(std::floor((z - margin) / kCell));
    const int cz1 = static_cast<int>(std::floor((z + margin) / kCell));
    for (int cx = cx0; cx <= cx1; ++cx)
        for (int cz = cz0; cz <= cz1; ++cz) {
            auto it = cells.find(cellKey(cx, cz));
            if (it == cells.end()) continue;
            for (int ti : it->second) {
                const Tri& t = tris[static_cast<std::size_t>(ti)];
                if (inTri(t, x, z) || segDist(p, t.a, t.b) < margin || segDist(p, t.b, t.c) < margin ||
                    segDist(p, t.c, t.a) < margin)
                    return true;
            }
        }
    return false;
}

DrawnRoad gatherDrawnRoad(World& world) {
    DrawnRoad rp;
    world.each<Renderable>([&](Entity e, Renderable& r) {
        if (!(r.renderLayer & LayerRoads)) return;
        const MeshCollider* mc = world.get<MeshCollider>(e);
        if (!mc) return;
        for (std::size_t i = 0; i + 2 < mc->indices.size(); i += 3) {
            const Vec3& a = mc->vertices[mc->indices[i]];
            const Vec3& b = mc->vertices[mc->indices[i + 1]];
            const Vec3& c = mc->vertices[mc->indices[i + 2]];
            rp.add(Vec2(a.x, a.z), Vec2(b.x, b.z), Vec2(c.x, c.z));
        }
    });
    return rp;
}

}  // namespace engine
