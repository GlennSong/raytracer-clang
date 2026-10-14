#ifndef RAYTRACER_ENGINE_MESH_AUDIT_H
#define RAYTRACER_ENGINE_MESH_AUDIT_H

#include "mesh_builder.h"
#include "../renderer/renderer.h"
#include "../rt_math.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace engine {

// THE MESH AUDIT (Glenn, 2026-10-13: "the entire procedural generation sits on this ... Everything, and I mean
// everything in this engine should use the same tooling"). Every producer's triangles judged by one rule set -- the
// unit tests (tests/test_winding.cpp, every producer) and the baked levels (rt_bake --audit, every mesh a level ships):
//
//   INWARD      the engine's front face is CLOCKWISE seen from the side the normal points to (VK_FRONT_FACE_CLOCKWISE,
//               back faces culled): (a, b, c) faces out when cross(c - a, b - a) runs along its vertex normals. One
//               that does not is drawn from the inside -- invisible from outside, a hole from below. Judged by ANGLE
//               (test_winding's rule since 554b3155): a true flip is ~180 deg apart (cos ~ -1); a surface-nets sliver
//               or a smoothed crease vertex sits near 90 deg and is not one, so the cut is cos < -0.3 (> 107 deg).
//   DEGENERATE  no area (MeshBuilder::kNoArea2: what the emitters themselves skip).
//   NON-FINITE  a NaN or infinite position, or an index past the vertices.
//   BAD NORMAL  a vertex normal not unit length (zero shades black and judges nothing).
struct MeshAudit {
    long tris = 0, inward = 0, degenerate = 0, nonFinite = 0, badNormals = 0;
    long flipped = 0;   // of `inward`, the ones wholly reversed (cos < -0.9): a wrong index order, not a crease
    Vec3 where{0, 0, 0};   // the first inward triangle's centroid (mesh space), to find it
    Vec3 whereN{0, 0, 0}, whereSize{0, 0, 0};   // its vertex normal (averaged) and its longest edge's vector
    double whereArea = 0;                       // and its area (m^2): a sliver flips on float rounding alone
    long inwardSliver = 0;                      // of `inward`, the ones thinner than 1 cm (area / longest edge)
    Vec3 degenWhere{0, 0, 0}, degenA, degenB, degenC;   // the first degenerate one's corners
    void add(const MeshAudit& o) {
        if (degenerate == 0 && o.degenerate > 0) { degenWhere = o.degenWhere; degenA = o.degenA; degenB = o.degenB; degenC = o.degenC; }
        if (inward == 0 && o.inward > 0) { where = o.where; whereN = o.whereN; whereSize = o.whereSize; whereArea = o.whereArea; }
        inwardSliver += o.inwardSliver;
        flipped += o.flipped;
        tris += o.tris; inward += o.inward; degenerate += o.degenerate; nonFinite += o.nonFinite; badNormals += o.badNormals;
    }
    bool clean() const { return inward == 0 && degenerate == 0 && nonFinite == 0 && badNormals == 0; }
};

inline MeshAudit auditMesh(const RenderMesh& m) {
    MeshAudit r;
    for (const Vertex& v : m.vertices) {
        const double nl = v.normal.length();
        if (!(nl > 0.5 && nl < 1.5)) ++r.badNormals;
    }
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const uint32_t ia = m.indices[t], ib = m.indices[t + 1], ic = m.indices[t + 2];
        ++r.tris;
        if (ia >= m.vertices.size() || ib >= m.vertices.size() || ic >= m.vertices.size()) { ++r.nonFinite; continue; }
        const Vertex& a = m.vertices[ia]; const Vertex& b = m.vertices[ib]; const Vertex& c = m.vertices[ic];
        bool finite = true;
        for (const Vertex* v : {&a, &b, &c})
            finite = finite && std::isfinite(v->position.x) && std::isfinite(v->position.y) && std::isfinite(v->position.z);
        if (!finite) { ++r.nonFinite; continue; }
        const Vec3 f = cross(c.position - a.position, b.position - a.position);
        if (ia == ib || ib == ic || ia == ic || f.lengthSquared() < MeshBuilder::kNoArea2) {
            if (r.degenerate++ == 0) {
                r.degenWhere = (a.position + b.position + c.position) / 3.0;
                r.degenA = a.position; r.degenB = b.position; r.degenC = c.position;
            }
            continue;
        }
        const Vec3 n = a.normal + b.normal + c.normal;
        const double cs = n.lengthSquared() > 1e-12 ? dot(normalize(f), normalize(n)) : 1.0;
        if (cs < -0.3) {
            const Vec3 e[3] = {b.position - a.position, c.position - b.position, a.position - c.position};
            Vec3 longest = e[0];
            for (const Vec3& x : e) if (x.lengthSquared() > longest.lengthSquared()) longest = x;
            const double area = 0.5 * f.length();
            if (area / std::max(1e-12, longest.length()) < 0.01) ++r.inwardSliver;
            if (r.inward++ == 0) {
                r.where = (a.position + b.position + c.position) / 3.0;
                r.whereN = normalize(n);
                r.whereSize = longest;
                r.whereArea = area;
            }
        }
        if (cs < -0.9) ++r.flipped;
    }
    return r;
}

inline void printMeshAudit(const char* what, const MeshAudit& a) {
    std::printf("    [mesh audit] %-40s %8ld tris %7ld inward (%ld flipped) %6ld degen %4ld nonfinite %6ld badnormal%s\n", what,
                a.tris, a.inward, a.flipped, a.degenerate, a.nonFinite, a.badNormals, a.clean() ? "" : "  <--");
    if (a.degenerate > 0)
        std::printf("    [mesh audit] %-40s first degenerate (%.3f %.3f %.3f) (%.3f %.3f %.3f) (%.3f %.3f %.3f)\n", "",
                    a.degenA.x, a.degenA.y, a.degenA.z, a.degenB.x, a.degenB.y, a.degenB.z, a.degenC.x, a.degenC.y, a.degenC.z);
    if (a.inward > 0)
        std::printf("    [mesh audit] %-40s first inward at %.2f %.2f %.2f, normal %.2f %.2f %.2f, longest edge %.2f m, area %.5f m2;"
                    " %ld of the inward are slivers (< 1 cm across)\n", "", a.where.x, a.where.y, a.where.z, a.whereN.x, a.whereN.y,
                    a.whereN.z, a.whereSize.length(), a.whereArea, a.inwardSliver);
}


// UNDERSIDES (Glenn, 2026-10-13: "the elevated freeway/rail undercarriage when viewed from below can be seen through").
// A surface you stand on that hangs more than `hang` over the ground must have something facing DOWN under it within
// `depth` -- its soffit, a wall's bottom, or a solid's bottom at or under the ground -- or from below you look at the back of the road (culled) and through it.
// Sampled at every up-facing triangle's centroid of the `walk` meshes; `ground(x, z)` the ground's height there (NaN:
// unknown, skipped). Every mesh in `all` may cover. Counts by material, and the open places by 100 m square.
struct UndersideAudit {
    size_t hanging = 0, open = 0;
    std::map<std::string, std::pair<size_t, size_t>> byMaterial;   // hanging, open
    std::map<std::pair<int, int>, size_t> openBySquare;            // floor(x / 100), floor(z / 100)
    Vec3 firstOpen{0, 0, 0};
};
struct NamedMeshRef { std::string name; const RenderMesh* mesh; };
inline UndersideAudit auditUndersides(const std::vector<NamedMeshRef>& all, const std::function<bool(const std::string&)>& walk,
                                      const std::function<double(double, double)>& ground, double hang = 2.0, double depth = 3.0) {
    struct Tri { Vec3 a, b, c; };
    auto under = [](const Tri& t, double x, double z, double& y) {
        const double d = (t.b.z - t.c.z) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.z - t.c.z);
        if (std::fabs(d) < 1e-12) return false;
        const double l1 = ((t.b.z - t.c.z) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.z)) / d;
        const double l2 = ((t.c.z - t.a.z) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.z)) / d;
        const double l3 = 1 - l1 - l2;
        if (l1 < -1e-6 || l2 < -1e-6 || l3 < -1e-6) return false;
        y = l1 * t.a.y + l2 * t.b.y + l3 * t.c.y;
        return true;
    };
    std::map<std::pair<int, int>, std::vector<Tri>> down;   // by 4 m bucket
    std::vector<std::pair<const std::string*, Tri>> tops;
    for (const NamedMeshRef& nm : all) {
        const RenderMesh& m = *nm.mesh;
        const bool isWalk = walk(nm.name);
        for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const Tri tri{m.vertices[m.indices[t]].position, m.vertices[m.indices[t + 1]].position, m.vertices[m.indices[t + 2]].position};
            const Vec3 f = cross(tri.c - tri.a, tri.b - tri.a);
            if (f.lengthSquared() < 1e-10) continue;
            const double ny = f.y / f.length();
            if (ny < -0.7) {
                const double x0 = std::min({tri.a.x, tri.b.x, tri.c.x}), x1 = std::max({tri.a.x, tri.b.x, tri.c.x});
                const double z0 = std::min({tri.a.z, tri.b.z, tri.c.z}), z1 = std::max({tri.a.z, tri.b.z, tri.c.z});
                for (int i = static_cast<int>(std::floor(x0 / 4)); i <= static_cast<int>(std::floor(x1 / 4)); ++i)
                    for (int k = static_cast<int>(std::floor(z0 / 4)); k <= static_cast<int>(std::floor(z1 / 4)); ++k) down[{i, k}].push_back(tri);
            } else if (isWalk && ny > 0.7) {
                tops.push_back({&nm.name, tri});
            }
        }
    }
    UndersideAudit r;
    for (const auto& [mat, t] : tops) {
        const Vec3 c = (t.a + t.b + t.c) / 3.0;
        const double gy = ground(c.x, c.z);
        if (!std::isfinite(gy) || c.y - gy < hang) continue;
        ++r.hanging;
        ++r.byMaterial[*mat].first;
        bool covered = false;
        double y;
        if (auto it = down.find({static_cast<int>(std::floor(c.x / 4)), static_cast<int>(std::floor(c.z / 4))}); it != down.end())
            for (const Tri& d : it->second)
                // its soffit (within `depth` under it, or level with it: a sheet), or the bottom of the solid it tops,
                // down at or under the ground (a pier's cap: nobody stands inside a pier to look up)
                if (under(d, c.x, c.z, y) && y < c.y + 0.05 && (y > c.y - depth || y < gy + 0.05)) { covered = true; break; }
        if (covered) continue;
        if (r.open++ == 0) r.firstOpen = c;
        ++r.byMaterial[*mat].second;
        ++r.openBySquare[{static_cast<int>(std::floor(c.x / 100)), static_cast<int>(std::floor(c.z / 100))}];
    }
    return r;
}

inline void printUndersideAudit(const UndersideAudit& u, size_t worst = 12) {
    std::printf("    [undersides] %zu walkable triangles hang over the ground, %zu with nothing facing down under them\n", u.hanging, u.open);
    for (const auto& [m, n] : u.byMaterial) std::printf("    [undersides]   %-12s %8zu hanging %6zu open\n", m.c_str(), n.first, n.second);
    std::vector<std::pair<size_t, std::pair<int, int>>> w;
    for (const auto& [k, n] : u.openBySquare) w.push_back({n, k});
    std::sort(w.rbegin(), w.rend());
    for (size_t i = 0; i < w.size() && i < worst; ++i)
        std::printf("    [undersides]   %5zu open around x %d, z %d\n", w[i].first, w[i].second.first * 100 + 50, w[i].second.second * 100 + 50);
}

// WHAT STANDS IN ONE COLUMN: every face within `radius` of (x, z), top down -- its mesh, centre and facing. The probe
// an audit miss is read with (rt_bake --audit's RT_AUDIT_PROBE, the lanes tests).
inline void printColumn(const std::vector<NamedMeshRef>& all, double x, double z, double radius = 1.5) {
    std::vector<std::pair<double, std::string>> column;
    for (const NamedMeshRef& nm : all)
        for (size_t t = 0; t + 2 < nm.mesh->indices.size(); t += 3) {
            const Vec3 &A = nm.mesh->vertices[nm.mesh->indices[t]].position, &B = nm.mesh->vertices[nm.mesh->indices[t + 1]].position,
                       &C = nm.mesh->vertices[nm.mesh->indices[t + 2]].position;
            const Vec3 c = (A + B + C) / 3.0, f = cross(C - A, B - A);
            if (std::hypot(c.x - x, c.z - z) > radius || f.lengthSquared() < 1e-12) continue;
            char line[200];
            std::snprintf(line, sizeof line, "%-12s centre %.2f %.2f %.2f facing %.2f %.2f %.2f", nm.name.c_str(), c.x, c.y, c.z,
                          f.x / f.length(), f.y / f.length(), f.z / f.length());
            column.push_back({c.y, line});
        }
    std::sort(column.rbegin(), column.rend());
    for (const auto& [y, l] : column) std::printf("    [column] %s\n", l.c_str());
}

}  // namespace engine

#endif
