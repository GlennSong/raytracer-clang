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

void DrawnRoad::add(const Vec3& a, const Vec3& b, const Vec3& c) {
    add(Vec2(a.x, a.z), Vec2(b.x, b.z), Vec2(c.x, c.z));
    heights.resize(tris.size());
    heights.back() = {static_cast<float>(a.y), static_cast<float>(b.y), static_cast<float>(c.y)};
}

bool DrawnRoad::topAt(double x, double z, double* y, double ceiling) const {
    auto it = cells.find(cellKey(static_cast<int>(std::floor(x / kCell)), static_cast<int>(std::floor(z / kCell))));
    if (it == cells.end()) return false;
    bool found = false;
    double best = -1e30;
    for (int ti : it->second) {
        if (static_cast<std::size_t>(ti) >= heights.size()) continue;   // added plan-only
        const Tri& t = tris[static_cast<std::size_t>(ti)];
        const double d = (t.b.y - t.c.y) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.y - t.c.y);
        if (std::fabs(d) < 1e-9) continue;   // a wall: no area in plan
        const double l1 = ((t.b.y - t.c.y) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.y)) / d;
        const double l2 = ((t.c.y - t.a.y) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.y)) / d;
        const double l3 = 1.0 - l1 - l2;
        if (l1 < -1e-6 || l2 < -1e-6 || l3 < -1e-6) continue;
        const std::array<float, 3>& h = heights[static_cast<std::size_t>(ti)];
        const double at = l1 * h[0] + l2 * h[1] + l3 * h[2];
        if (at > ceiling || at <= best) continue;
        best = at;
        found = true;
    }
    if (found && y) *y = best;
    return found;
}

bool DrawnRoad::overhead(double x, double z, double footY, double topY, double radius) const {
    const Vec2 p(x, z);
    const int cx0 = static_cast<int>(std::floor((x - radius) / kCell)), cx1 = static_cast<int>(std::floor((x + radius) / kCell));
    const int cz0 = static_cast<int>(std::floor((z - radius) / kCell)), cz1 = static_cast<int>(std::floor((z + radius) / kCell));
    for (int cx = cx0; cx <= cx1; ++cx)
        for (int cz = cz0; cz <= cz1; ++cz) {
            auto it = cells.find(cellKey(cx, cz));
            if (it == cells.end()) continue;
            for (int ti : it->second) {
                if (static_cast<std::size_t>(ti) >= heights.size()) continue;
                const std::array<float, 3>& h = heights[static_cast<std::size_t>(ti)];
                const double lo = std::min({h[0], h[1], h[2]}), hi = std::max({h[0], h[1], h[2]});
                if (hi < footY + 0.5 || lo > topY) continue;   // under the foot (its own pavement), or clear above
                const Tri& t = tris[static_cast<std::size_t>(ti)];
                if (inTri(t, x, z) || segDist(p, t.a, t.b) < radius || segDist(p, t.b, t.c) < radius || segDist(p, t.c, t.a) < radius)
                    return true;
            }
        }
    return false;
}

void DrawnRoad::addRail(const Vec2& a, const Vec2& b, double ya, double yb, double half) {
    RailRun r;
    r.a = a; r.b = b;
    r.ya = static_cast<float>(ya); r.yb = static_cast<float>(yb);
    r.half = static_cast<float>(std::min(4.0, half));   // (a track's own width; a wide spine is bounded by the query's reach)
    const int idx = static_cast<int>(rails.size());
    rails.push_back(r);
    const int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) / kCell)), x1 = static_cast<int>(std::floor(std::max(a.x, b.x) / kCell));
    const int z0 = static_cast<int>(std::floor(std::min(a.y, b.y) / kCell)), z1 = static_cast<int>(std::floor(std::max(a.y, b.y) / kCell));
    for (int cx = x0; cx <= x1; ++cx)
        for (int cz = z0; cz <= z1; ++cz) railCells[cellKey(cx, cz)].push_back(idx);
}

bool DrawnRoad::inTrainEnvelope(double x, double z, double footY, double topY, double radius) const {
    if (rails.empty()) return false;
    const Vec2 p(x, z);
    const double reach = radius + kTrainHalfWidth + kRailClearance + 4.0;   // (+ a track's own half-width, bounded)
    const int cx0 = static_cast<int>(std::floor((x - reach) / kCell)), cx1 = static_cast<int>(std::floor((x + reach) / kCell));
    const int cz0 = static_cast<int>(std::floor((z - reach) / kCell)), cz1 = static_cast<int>(std::floor((z + reach) / kCell));
    for (int cx = cx0; cx <= cx1; ++cx)
        for (int cz = cz0; cz <= cz1; ++cz) {
            auto it = railCells.find(cellKey(cx, cz));
            if (it == railCells.end()) continue;
            for (int ri : it->second) {
                const RailRun& r = rails[static_cast<std::size_t>(ri)];
                const Vec2 ab = r.b - r.a;
                const double l2 = ab.lengthSquared();
                const double t = l2 > 1e-12 ? std::clamp(dot(p - r.a, ab) / l2, 0.0, 1.0) : 0.0;
                if ((p - (r.a + ab * t)).length() >= r.half + kTrainHalfWidth + kRailClearance + radius) continue;
                const double y = r.ya + (r.yb - r.ya) * t;
                if (topY < y || footY > y + kTrainHeight) continue;   // wholly under the rails (a viaduct's) or over the train
                return true;
            }
        }
    return false;
}

bool DrawnRoad::blocksTree(double x, double z, double footY, double height, double crown) const {
    return overhead(x, z, footY, footY + height, crown) || inTrainEnvelope(x, z, footY, footY + height, crown);
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

DrawnRoad gatherDrawnRoad(World& world, bool withHeights) {
    DrawnRoad rp;
    // the tracks the city published, for the train's room (always: a few hundred segments)
    world.each<RoadDeck>([&](Entity, RoadDeck& d) {
        for (const UnionSpine& sp : d.field.spines) {
            if (sp.use != UnionSpine::Use::Track || sp.points.size() < 2 || sp.yAbs.size() != sp.points.size()) continue;
            for (std::size_t i = 0; i + 1 < sp.points.size(); ++i)
                rp.addRail(sp.points[i], sp.points[i + 1], sp.yAbs[i], sp.yAbs[i + 1], sp.halfWidth);
        }
    });
    world.each<Renderable>([&](Entity e, Renderable& r) {
        if (!(r.renderLayer & LayerRoads)) return;
        const MeshCollider* mc = world.get<MeshCollider>(e);
        if (!mc) return;
        for (std::size_t i = 0; i + 2 < mc->indices.size(); i += 3) {
            const Vec3& a = mc->vertices[mc->indices[i]];
            const Vec3& b = mc->vertices[mc->indices[i + 1]];
            const Vec3& c = mc->vertices[mc->indices[i + 2]];
            if (withHeights)
                rp.add(a, b, c);
            else
                rp.add(Vec2(a.x, a.z), Vec2(b.x, b.z), Vec2(c.x, c.z));
        }
    });
    return rp;
}

}  // namespace engine
