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
#include <set>
#include <string>
#include <tuple>
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
    struct Open { std::string mesh; Vec3 at; double ground; Vec3 a, b, c; };
    std::vector<Open> all;   // every open top's centre, for sampling a cluster (rt_bake RT_UNDERSIDE_SAMPLES)
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
        r.all.push_back({*mat, c, gy, t.a, t.b, t.c});
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

// EVERY FACE ALONG ONE RAY o + d t, t in (0, reach), nearest first: its mesh, distance, hit point, facing and whether
// the ray meets its FRONT. The probe a see-through look is read with (rt_bake --audit's RT_AUDIT_RAY) -- a column
// cannot show a side face merged into one long quad, whose centre stands metres away.
inline void printRay(const std::vector<NamedMeshRef>& all, const Vec3& o, Vec3 d, double reach = 40.0) {
    d = normalize(d);
    std::vector<std::pair<double, std::string>> hits;
    for (const NamedMeshRef& nm : all)
        for (size_t t = 0; t + 2 < nm.mesh->indices.size(); t += 3) {
            const Vec3 &A = nm.mesh->vertices[nm.mesh->indices[t]].position, &B = nm.mesh->vertices[nm.mesh->indices[t + 1]].position,
                       &C = nm.mesh->vertices[nm.mesh->indices[t + 2]].position;
            const Vec3 e1 = B - A, e2 = C - A, p = cross(d, e2);
            const double det = dot(e1, p);
            if (std::fabs(det) < 1e-12) continue;
            const Vec3 s = o - A;
            const double u = dot(s, p) / det;
            if (u < 0 || u > 1) continue;
            const Vec3 q = cross(s, e1);
            const double v = dot(d, q) / det;
            if (v < 0 || u + v > 1) continue;
            const double tt = dot(e2, q) / det;
            if (tt <= 1e-4 || tt > reach) continue;
            const Vec3 f = cross(C - A, B - A), h = o + d * tt;
            char line[220];
            std::snprintf(line, sizeof line, "%-12s t %6.2f at %.2f %.2f %.2f facing %.2f %.2f %.2f %s", nm.name.c_str(), tt, h.x, h.y, h.z,
                          f.x / f.length(), f.y / f.length(), f.z / f.length(), dot(f, d) < 0 ? "FRONT" : "back");
            hits.push_back({tt, line});
        }
    std::sort(hits.begin(), hits.end());
    for (const auto& [t, l] : hits) std::printf("    [ray] %s\n", l.c_str());
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

// CLOSED? The edges of `m`, welded by position (1 mm), used by other than exactly two triangles -- 0 for a closed
// solid (a box, a block, a prism, a capped cylinder). The library's solids are closed (mesh_builder.h).
inline size_t openEdges(const RenderMesh& m) {
    auto key = [](const Vec3& p) {
        return std::make_tuple(static_cast<long long>(std::llround(p.x * 1000)), static_cast<long long>(std::llround(p.y * 1000)),
                               static_cast<long long>(std::llround(p.z * 1000)));
    };
    std::map<std::tuple<long long, long long, long long>, int> ids;
    auto id = [&](const Vec3& p) { return ids.emplace(key(p), static_cast<int>(ids.size())).first->second; };
    std::map<std::pair<int, int>, int> uses;
    for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const int v[3] = {id(m.vertices[m.indices[t]].position), id(m.vertices[m.indices[t + 1]].position), id(m.vertices[m.indices[t + 2]].position)};
        for (int k = 0; k < 3; ++k) ++uses[{std::min(v[k], v[(k + 1) % 3]), std::max(v[k], v[(k + 1) % 3])}];
    }
    size_t open = 0;
    for (const auto& [e, n] : uses) if (n != 2) ++open;
    return open;
}

// A CROSS-SECTION: walk from (x0, z0) to (x1, z1) in `step`s and, at each point, list every face whose plan covers it
// -- its mesh, its height, and whether it faces up (^), down (v) or sideways (|). A crack in a deck is a point under
// it with nothing overhead; a hole in its soffit, an up face with no v under it. Not for every mesh: a probe.
inline void printSection(const std::vector<NamedMeshRef>& all, double x0, double z0, double x1, double z1, double step) {
    const double len = std::hypot(x1 - x0, z1 - z0);
    const int n = std::max(1, static_cast<int>(len / step));
    for (int i = 0; i <= n; ++i) {
        const double x = x0 + (x1 - x0) * i / n, z = z0 + (z1 - z0) * i / n;
        std::vector<std::pair<double, std::string>> hits;
        for (const NamedMeshRef& nm : all) {
            const RenderMesh& m = *nm.mesh;
            for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
                const Vec3 &A = m.vertices[m.indices[t]].position, &B = m.vertices[m.indices[t + 1]].position,
                           &C = m.vertices[m.indices[t + 2]].position;
                if (std::max({A.x, B.x, C.x}) < x || std::min({A.x, B.x, C.x}) > x || std::max({A.z, B.z, C.z}) < z ||
                    std::min({A.z, B.z, C.z}) > z) continue;
                const double d = (B.z - C.z) * (A.x - C.x) + (C.x - B.x) * (A.z - C.z);
                if (std::fabs(d) < 1e-12) continue;
                const double l1 = ((B.z - C.z) * (x - C.x) + (C.x - B.x) * (z - C.z)) / d;
                const double l2 = ((C.z - A.z) * (x - C.x) + (A.x - C.x) * (z - C.z)) / d;
                if (l1 < -1e-6 || l2 < -1e-6 || 1 - l1 - l2 < -1e-6) continue;
                const Vec3 f = cross(C - A, B - A);
                const double ny = f.y / std::max(1e-12, f.length());
                char buf[64];
                std::snprintf(buf, sizeof buf, "%s%c%.2f", nm.name.c_str(), ny > 0.3 ? '^' : ny < -0.3 ? 'v' : '|',
                              l1 * A.y + l2 * B.y + (1 - l1 - l2) * C.y);
                hits.push_back({l1 * A.y + l2 * B.y + (1 - l1 - l2) * C.y, buf});
            }
        }
        std::sort(hits.rbegin(), hits.rend());
        std::printf("    [section] %8.2f %8.2f :", x, z);
        for (const auto& [y, h] : hits) std::printf(" %s", h.c_str());
        std::printf("\n");
    }
}

// SEE-THROUGH FROM BELOW (Glenn, 2026-10-14: "a strip of polygons missing along the side ... the hole is clearly one
// way"): stand on the ground under every hanging walkable surface (one eye per 2 m) and look up -- straight up and at 45 deg in
// eight directions. The first face each ray meets must face the eye; one that faces away is the back of a surface
// (culled), so the eye sees through it: a missing side or soffit. Rays that meet nothing within `reach` are open sky
// beside the deck, which is fine. Counts the looks that see through, and where the first one was.
struct SeeThroughAudit {
    size_t eyes = 0, looks = 0, through = 0, insideSolid = 0;   // insideSolid: eyes skipped, inside a girder or pier
    Vec3 firstEye{0, 0, 0}, firstDir{0, 0, 0};
    std::map<std::string, size_t> byHitMesh;          // the mesh whose back the eye saw
    std::vector<std::string> samples;                 // the first few, described
    std::map<std::pair<int, int>, size_t> bySquare;   // floor(x / 100), floor(z / 100)
    struct Look { Vec3 eye, dir, hit, facing; double deckTop; std::string mesh; };
    std::vector<Look> all;                            // every see-through look
};
inline SeeThroughAudit auditSeeThrough(const std::vector<NamedMeshRef>& all, const std::function<bool(const std::string&)>& walk,
                                       const std::function<double(double, double)>& ground, double hang = 2.0, double reach = 80.0) {
    struct Tri { Vec3 a, b, c; const std::string* mesh; size_t id; };
    std::map<std::pair<int, int>, std::vector<Tri>> grid;   // every triangle by 4 m bucket (plan bbox)
    size_t nextId = 0;
    std::vector<Vec3> tops;
    for (const NamedMeshRef& nm : all) {
        const RenderMesh& m = *nm.mesh;
        const bool isWalk = walk(nm.name);
        for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const Tri tri{m.vertices[m.indices[t]].position, m.vertices[m.indices[t + 1]].position, m.vertices[m.indices[t + 2]].position, &nm.name, nextId++};
            const Vec3 f = cross(tri.c - tri.a, tri.b - tri.a);
            if (f.lengthSquared() < 1e-12) continue;
            for (int i = static_cast<int>(std::floor(std::min({tri.a.x, tri.b.x, tri.c.x}) / 4)); i <= static_cast<int>(std::floor(std::max({tri.a.x, tri.b.x, tri.c.x}) / 4)); ++i)
                for (int k = static_cast<int>(std::floor(std::min({tri.a.z, tri.b.z, tri.c.z}) / 4)); k <= static_cast<int>(std::floor(std::max({tri.a.z, tri.b.z, tri.c.z}) / 4)); ++k)
                    grid[{i, k}].push_back(tri);
            if (isWalk && f.y / f.length() > 0.7) tops.push_back((tri.a + tri.b + tri.c) / 3.0);
        }
    }
    // nearest hit along o + d t, t in (eps, reach): its triangle's facing (cross(c-a, b-a)), or zero for none
    const std::string* hitMesh = nullptr;
    auto cast = [&](const Vec3& o, const Vec3& d, double& tHit) {
        Vec3 face(0, 0, 0);
        tHit = reach;
        const Vec3 e = o + d * reach;
        const int i0 = static_cast<int>(std::floor(std::min(o.x, e.x) / 4)), i1 = static_cast<int>(std::floor(std::max(o.x, e.x) / 4));
        const int k0 = static_cast<int>(std::floor(std::min(o.z, e.z) / 4)), k1 = static_cast<int>(std::floor(std::max(o.z, e.z) / 4));
        for (int i = i0; i <= i1; ++i)
            for (int k = k0; k <= k1; ++k) {
                auto it = grid.find({i, k});
                if (it == grid.end()) continue;
                for (const Tri& t : it->second) {   // Moller-Trumbore, two-sided
                    const Vec3 e1 = t.b - t.a, e2 = t.c - t.a, p = cross(d, e2);
                    const double det = dot(e1, p);
                    if (std::fabs(det) < 1e-12) continue;
                    const double inv = 1.0 / det;
                    const Vec3 s = o - t.a;
                    const double u = dot(s, p) * inv;
                    if (u < 0 || u > 1) continue;
                    const Vec3 q = cross(s, e1);
                    const double v = dot(d, q) * inv;
                    if (v < 0 || u + v > 1) continue;
                    const double tt = dot(e2, q) * inv;
                    if (tt > 1e-4 && tt < tHit) { tHit = tt; face = cross(t.c - t.a, t.b - t.a); hitMesh = t.mesh; }
                }
            }
        return face;
    };
    // THE WINDING along o + d t, t in (eps, reach): +1 for every back the ray crosses (leaving a solid), -1 for every
    // front (entering one). Above zero, o is inside something -- and unlike "the first face met is a back" it stays
    // right where closed solids overlap: a pier standing up into a box girder's webs meets a web's FRONT first
    // from inside the pier (the island's see-through samples, 2026-10-14).
    auto winding = [&](const Vec3& o, const Vec3& d) {
        int w = 0;
        std::set<size_t> met;   // a triangle spans several buckets
        const Vec3 e = o + d * reach;
        const int i0 = static_cast<int>(std::floor(std::min(o.x, e.x) / 4)), i1 = static_cast<int>(std::floor(std::max(o.x, e.x) / 4));
        const int k0 = static_cast<int>(std::floor(std::min(o.z, e.z) / 4)), k1 = static_cast<int>(std::floor(std::max(o.z, e.z) / 4));
        for (int i = i0; i <= i1; ++i)
            for (int k = k0; k <= k1; ++k) {
                auto it = grid.find({i, k});
                if (it == grid.end()) continue;
                for (const Tri& t : it->second) {
                    const Vec3 e1 = t.b - t.a, e2 = t.c - t.a, p = cross(d, e2);
                    const double det = dot(e1, p);
                    if (std::fabs(det) < 1e-12) continue;
                    const double inv = 1.0 / det;
                    const Vec3 s = o - t.a;
                    const double u = dot(s, p) * inv;
                    if (u < 0 || u > 1) continue;
                    const Vec3 q = cross(s, e1);
                    const double v = dot(d, q) * inv;
                    if (v < 0 || u + v > 1) continue;
                    const double tt = dot(e2, q) * inv;
                    if (tt <= 1e-4 || tt >= reach || !met.insert(t.id).second) continue;
                    w += dot(cross(t.c - t.a, t.b - t.a), d) > 0 ? 1 : -1;
                }
            }
        return w;
    };
    SeeThroughAudit r;
    std::vector<Vec3> dirs;   // (straight up is the eye test below)
    for (int k = 0; k < 8; ++k) {
        const double a = 6.283185307179586 * k / 8;
        dirs.push_back(normalize(Vec3(std::cos(a), 1.0, std::sin(a))));
    }
    std::map<std::tuple<long, long, long>, bool> seen;   // one eye per 1 m cell
    for (const Vec3& c : tops) {
        const double gy = ground(c.x, c.z);
        if (!std::isfinite(gy) || c.y - gy < hang) continue;
        // a PERSON standing on the ground under it (eye height 1.7 m): what Glenn looked up from
        const Vec3 eye(c.x, gy + 1.7, c.z);
        if (!seen.emplace(std::make_tuple(std::lround(eye.x / 2), std::lround(eye.y / 2), std::lround(eye.z / 2)), true).second) continue;
        // an eye INSIDE a solid (a girder hung under the slab, a pier) sees every wall from behind: it is not somewhere
        // anyone stands. From inside a closed solid the first face met is a back, at any distance -- so most of eight
        // level looks meeting a back, or the look straight down meeting one, says so. (A 3 m limit on the level looks
        // let the eye stand inside a pier longer than that, whose head pokes up through the soffit: the interchange.)
        int enclosed = 0;
        for (int k = 0; k < 8; ++k) {
            const double a = 6.283185307179586 * k / 8;
            const Vec3 d(std::cos(a), 0, std::sin(a));
            double t;
            const Vec3 face = cast(eye, d, t);
            if ((face.lengthSquared() > 0 && dot(face, d) > 0) || winding(eye, d) > 0) ++enclosed;
        }
        bool floorBehind = false;
        {
            double t;
            const Vec3 face = cast(eye, Vec3(0, -1, 0), t);
            floorBehind = face.lengthSquared() > 0 && face.y < 0;   // the first face below faces down: inside
        }
        if (enclosed >= 5 || floorBehind) { ++r.insideSolid; continue; }
        // ...and one with no HEADROOM is in a slab or a plinth, or under a deck too low to stand under: straight up from
        // the ground, the first thing met must be the FRONT of a soffit at least half a metre over the eye (a missing
        // soffit overhead is auditUndersides'). From the ground, not the eye: a ramp end 1.3 m over the terrain puts
        // the eye inside its slab, where a ray up a seam can catch the next slab's soffit and pass.
        {
            constexpr double kFoot = 0.3;   // off the drawn ground, clear of the terrain mesh's own rounding
            double t;
            const Vec3 face = cast(Vec3(c.x, gy + kFoot, c.z), Vec3(0, 1, 0), t);
            if (face.lengthSquared() == 0 || dot(face, Vec3(0, 1, 0)) > 0 || t < 1.7 + 0.5 - kFoot) { ++r.insideSolid; continue; }
        }
        ++r.eyes;
        for (const Vec3& d : dirs) {
            ++r.looks;
            double t;
            const Vec3 face = cast(eye, d, t);
            if (face.lengthSquared() == 0) continue;
            if (dot(face, d) > 0) {   // its front faces along the ray: the eye sees its back
                if (r.through++ == 0) { r.firstEye = eye; r.firstDir = d; }
                ++r.byHitMesh[hitMesh ? *hitMesh : std::string("?")];
                if (r.samples.size() < 12) {
                    const Vec3 h = eye + d * t, fn = normalize(face);
                    char buf[200];
                    std::snprintf(buf, sizeof buf, "eye %.1f %.1f %.1f (deck top %.1f) dir %.2f %.2f %.2f hits %s at %.2f %.2f %.2f, %.2f m, facing %.2f %.2f %.2f",
                                  eye.x, eye.y, eye.z, c.y, d.x, d.y, d.z, hitMesh ? hitMesh->c_str() : "?", h.x, h.y, h.z, t, fn.x, fn.y, fn.z);
                    r.samples.push_back(buf);
                }
                ++r.bySquare[{static_cast<int>(std::floor(eye.x / 100)), static_cast<int>(std::floor(eye.z / 100))}];
                r.all.push_back({eye, d, eye + d * t, normalize(face), c.y, hitMesh ? *hitMesh : std::string("?")});
            }
        }
    }
    return r;
}

// FACADE CRACKS (Glenn, 2026-10-16: "some of the twisting buildings have cracks along the edges ... write something
// that checks buildings for facade cracks"). A building's outside is closed: a level ray from outside toward its axis
// must meet a FRONT face first. One whose first face is a BACK came in through a gap -- a crack between two storeys'
// faces, at a corner, or a hole -- and is looking at the inside of the far wall. Rays at every `dy` from y0 to y1, at
// `azimuths` bearings round (cx, cz), from `radius` out. A ray that meets nothing is above or beside the building.
struct FacadeCrackAudit {
    size_t rays = 0, hits = 0, cracks = 0;
    struct Crack { Vec3 from, dir, at; std::string mesh; double t; };
    std::vector<Crack> all;
    std::map<long, size_t> byStorey;   // floor(y / storeyH): which storeys leak
};
inline FacadeCrackAudit auditFacadeCracks(const std::vector<NamedMeshRef>& parts, double cx, double cz, double radius,
                                          double y0, double y1, double dy = 0.25, int azimuths = 1440, double storeyH = 3.6) {
    struct Tri { Vec3 a, b, c; const std::string* mesh; };
    std::map<long, std::vector<Tri>> band;   // every triangle by the 1 m bands of height it spans (the rays are level)
    for (const NamedMeshRef& nm : parts) {
        const RenderMesh& m = *nm.mesh;
        for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const Tri tri{m.vertices[m.indices[t]].position, m.vertices[m.indices[t + 1]].position, m.vertices[m.indices[t + 2]].position, &nm.name};
            if (cross(tri.c - tri.a, tri.b - tri.a).lengthSquared() < 1e-12) continue;
            const long b0 = static_cast<long>(std::floor(std::min({tri.a.y, tri.b.y, tri.c.y}))), b1 = static_cast<long>(std::floor(std::max({tri.a.y, tri.b.y, tri.c.y})));
            for (long b = b0; b <= b1; ++b) band[b].push_back(tri);
        }
    }
    FacadeCrackAudit r;
    for (double y = y0; y <= y1; y += dy) {
        auto it = band.find(static_cast<long>(std::floor(y)));
        if (it == band.end()) continue;
        for (int k = 0; k < azimuths; ++k) {
            const double a = 6.283185307179586 * (k + 0.5) / azimuths;
            const Vec3 d(-std::cos(a), 0, -std::sin(a));
            const Vec3 o(cx - d.x * radius, y, cz - d.z * radius);
            ++r.rays;
            double best = 2 * radius; const Tri* hit = nullptr;
            for (const Tri& t : it->second) {   // Moller-Trumbore, two-sided
                const Vec3 e1 = t.b - t.a, e2 = t.c - t.a, p = cross(d, e2);
                const double det = dot(e1, p);
                if (std::fabs(det) < 1e-12) continue;
                const double inv = 1.0 / det; const Vec3 s = o - t.a;
                const double u = dot(s, p) * inv; if (u < 0 || u > 1) continue;
                const Vec3 q = cross(s, e1);
                const double v = dot(d, q) * inv; if (v < 0 || u + v > 1) continue;
                const double tt = dot(e2, q) * inv;
                if (tt > 1e-4 && tt < best) { best = tt; hit = &t; }
            }
            if (!hit) continue;
            ++r.hits;
            if (dot(cross(hit->c - hit->a, hit->b - hit->a), d) <= 0) continue;   // a front: closed here
            ++r.cracks; ++r.byStorey[static_cast<long>(std::floor(y / storeyH))];
            if (r.all.size() < 4000) r.all.push_back({o, d, o + d * best, *hit->mesh, best});
        }
    }
    return r;
}

}  // namespace engine

#endif
