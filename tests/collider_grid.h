#ifndef RT_TESTS_COLLIDER_GRID_H
#define RT_TESTS_COLLIDER_GRID_H
// The static colliders as triangles, for tests that ask "what surface is under this point": the level suite's
// traffic-on-the-deck gate and the physical tier's deck soak (#24) share it.
#include "../src/engine/components.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace rt_test {
using engine::Vec3;
using engine::MeshCollider;

inline bool pointInTriangleXZ(double px, double pz, const Vec3& a, const Vec3& b,
                       const Vec3& c) {
    const double d1 = (px - b.x) * (a.z - b.z) - (a.x - b.x) * (pz - b.z);
    const double d2 = (px - c.x) * (b.z - c.z) - (b.x - c.x) * (pz - c.z);
    const double d3 = (px - a.x) * (c.z - a.z) - (c.x - a.x) * (pz - a.z);
    const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}

// Collider triangles bucketed by XZ cell so a probe touches a few dozen
// triangles, not the city's millions.
struct ColliderGrid {
    struct Tri { Vec3 a, b, c; };
    std::vector<Tri> tris;
    std::unordered_map<long long, std::vector<int>> cells;
    double lowest = 1e30;   // the lowest collider point (under it is the kill box)
    static constexpr double kCell = 12.0;
    static long long key(int cx, int cz) {
        return (static_cast<long long>(cx) << 32) ^ (static_cast<long long>(cz) & 0xffffffffLL);
    }
    void add(const MeshCollider& mc, const Vec3& at = Vec3(0, 0, 0)) {
        for (std::size_t i = 0; i + 2 < mc.indices.size(); i += 3) {
            Tri t{mc.vertices[mc.indices[i]] + at, mc.vertices[mc.indices[i + 1]] + at,
                  mc.vertices[mc.indices[i + 2]] + at};
            lowest = std::min({lowest, double(t.a.y), double(t.b.y), double(t.c.y)});
            const int id = static_cast<int>(tris.size());
            tris.push_back(t);
            const double x0 = std::min({t.a.x, t.b.x, t.c.x}), x1 = std::max({t.a.x, t.b.x, t.c.x});
            const double z0 = std::min({t.a.z, t.b.z, t.c.z}), z1 = std::max({t.a.z, t.b.z, t.c.z});
            for (int cz = static_cast<int>(std::floor(z0 / kCell)); cz <= static_cast<int>(std::floor(z1 / kCell)); ++cz)
                for (int cx = static_cast<int>(std::floor(x0 / kCell)); cx <= static_cast<int>(std::floor(x1 / kCell)); ++cx)
                    cells[key(cx, cz)].push_back(id);
        }
    }
    // Every collider surface under (x, z), for the offender listing.
    std::vector<double> surfacesAt(double x, double z) const {
        std::vector<double> out;
        auto it = cells.find(key(static_cast<int>(std::floor(x / kCell)),
                                 static_cast<int>(std::floor(z / kCell))));
        if (it == cells.end()) return out;
        for (int id : it->second) {
            const Tri& t = tris[id];
            if (!pointInTriangleXZ(x, z, t.a, t.b, t.c)) continue;
            const double den = (t.b.z - t.c.z) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.z - t.c.z);
            if (std::fabs(den) < 1e-12) continue;
            const double w0 = ((t.b.z - t.c.z) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.z)) / den;
            const double w1 = ((t.c.z - t.a.z) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.z)) / den;
            out.push_back(w0 * t.a.y + w1 * t.b.y + (1.0 - w0 - w1) * t.c.y);
        }
        std::sort(out.begin(), out.end());
        return out;
    }
    // Highest collider surface under (x, z) that is not above `ceiling`.
    bool surfaceAt(double x, double z, double ceiling, double& top) const {
        auto it = cells.find(key(static_cast<int>(std::floor(x / kCell)),
                                 static_cast<int>(std::floor(z / kCell))));
        if (it == cells.end()) return false;
        bool found = false;
        for (int id : it->second) {
            const Tri& t = tris[id];
            if (!pointInTriangleXZ(x, z, t.a, t.b, t.c)) continue;
            const double den = (t.b.z - t.c.z) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.z - t.c.z);
            if (std::fabs(den) < 1e-12) continue;
            const double w0 = ((t.b.z - t.c.z) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.z)) / den;
            const double w1 = ((t.c.z - t.a.z) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.z)) / den;
            const double y = w0 * t.a.y + w1 * t.b.y + (1.0 - w0 - w1) * t.c.y;
            if (y > ceiling) continue;
            if (!found || y > top) { top = y; found = true; }
        }
        return found;
    }
};

}  // namespace rt_test

#endif
