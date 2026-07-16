#pragma once
// Jolt-free geometric invariants for procedural meshes — the acceptance harness
// for the road/junction weld (road-unification-plan "Testing the viability of
// each join"). A weld regression (degenerate triangle, hole, flipped face) is
// otherwise invisible until seen by eye; these assert it mechanically over the
// output of buildRoadNetMesh / weldSolid for a matrix of join fixtures.

#include "../src/renderer/renderer.h"
#include "../src/rt_math.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace mesh_invariants {
using namespace engine;   // RenderMesh, Vertex, Vec3, cross

inline int triangleCount(const RenderMesh& m) {
    return static_cast<int>(m.indices.size() / 3);
}

// Triangles with a repeated index or a ~zero area — a triangulation/weld defect
// (the guards at road_mesh.cpp:252,1650 are never asserted in the shipping code).
inline int degenerateTriangles(const RenderMesh& m, double areaEps = 1e-7) {
    int bad = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        uint32_t a = m.indices[t], b = m.indices[t + 1], c = m.indices[t + 2];
        if (a == b || b == c || a == c) { ++bad; continue; }
        if (a >= m.vertices.size() || b >= m.vertices.size() || c >= m.vertices.size()) {
            ++bad; continue;
        }
        const Vec3& pa = m.vertices[a].position;
        const Vec3& pb = m.vertices[b].position;
        const Vec3& pc = m.vertices[c].position;
        // area = 0.5 * |(pb-pa) x (pc-pa)|
        const double area2 = cross(pb - pa, pc - pa).length();
        if (area2 < areaEps * 2.0) ++bad;
    }
    return bad;
}

// Any non-finite (NaN/Inf) vertex position — a numeric blow-up in the weld.
inline bool hasNonFinite(const RenderMesh& m) {
    for (const Vertex& v : m.vertices) {
        if (!std::isfinite(v.position.x) || !std::isfinite(v.position.y) ||
            !std::isfinite(v.position.z))
            return true;
    }
    return false;
}

// Every index refers to a real vertex.
inline bool indicesInRange(const RenderMesh& m) {
    for (uint32_t i : m.indices)
        if (i >= m.vertices.size()) return false;
    return true;
}

// XZ bounding box of the mesh (roads live near the plane; used to bound blow-ups
// where a weld throws a vertex to infinity-ish).
inline void bboxXZ(const RenderMesh& m, double& minX, double& maxX,
                   double& minZ, double& maxZ) {
    minX = minZ = 1e30; maxX = maxZ = -1e30;
    for (const Vertex& v : m.vertices) {
        minX = std::min(minX, static_cast<double>(v.position.x));
        maxX = std::max(maxX, static_cast<double>(v.position.x));
        minZ = std::min(minZ, static_cast<double>(v.position.z));
        maxZ = std::max(maxZ, static_cast<double>(v.position.z));
    }
}

inline void bboxY(const RenderMesh& m, double& minY, double& maxY) {
    minY = 1e30; maxY = -1e30;
    for (const Vertex& v : m.vertices) {
        minY = std::min(minY, static_cast<double>(v.position.y));
        maxY = std::max(maxY, static_cast<double>(v.position.y));
    }
}

// Fraction of triangles whose surface is oriented UP by its VERTEX normals (the
// mesh lights single-sided off vertex normals, so geometric winding is not the
// orientation of record). A road deck + its markings are strongly up-facing; if
// this collapses toward 0 the deck was flipped. A tri counts as "up" when its
// three vertex normals average to y > 0.7.
inline double upwardFraction(const RenderMesh& m) {
    int up = 0, tot = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        uint32_t a = m.indices[t], b = m.indices[t + 1], c = m.indices[t + 2];
        if (a >= m.vertices.size() || b >= m.vertices.size() || c >= m.vertices.size())
            continue;
        ++tot;
        const double ny = (m.vertices[a].normal.y + m.vertices[b].normal.y +
                           m.vertices[c].normal.y) / 3.0;
        if (ny > 0.7) ++up;
    }
    return tot > 0 ? static_cast<double>(up) / tot : 0.0;
}

// Count vertices inside an axis-aligned box (world space). The grade-sep
// "no curtain wall" check: a deck flying over a street must have NO geometry in
// the mid-height band directly over the crossing.
inline int verticesInBox(const RenderMesh& m, double x0, double x1, double y0,
                         double y1, double z0, double z1) {
    int n = 0;
    for (const Vertex& v : m.vertices) {
        const double x = v.position.x, y = v.position.y, z = v.position.z;
        if (x >= x0 && x <= x1 && y >= y0 && y <= y1 && z >= z0 && z <= z1) ++n;
    }
    return n;
}

// The Y heights of every UP-facing triangle whose XZ projection covers (x,z),
// sorted ascending. At a clean grade-separated crossing this returns TWO hits —
// the street (~grade) and the deck (~+9) — with clear air between; a merged
// (curtain-wall) weld returns a continuous ramp of hits instead. `upOnly` gates
// on averaged vertex-normal y so only drivable/deck faces count (not undersides).
inline std::vector<double> surfaceHitsAt(const RenderMesh& m, double x, double z,
                                         double upThresh = 0.5) {
    std::vector<double> hits;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        uint32_t ia = m.indices[t], ib = m.indices[t + 1], ic = m.indices[t + 2];
        if (ia >= m.vertices.size() || ib >= m.vertices.size() || ic >= m.vertices.size())
            continue;
        const Vertex& A = m.vertices[ia]; const Vertex& B = m.vertices[ib];
        const Vertex& C = m.vertices[ic];
        const double ny = (A.normal.y + B.normal.y + C.normal.y) / 3.0;
        if (ny < upThresh) continue;                 // not an up-facing surface
        // Barycentric test in the XZ plane.
        const double ax = A.position.x, az = A.position.z;
        const double bx = B.position.x, bz = B.position.z;
        const double cx = C.position.x, cz = C.position.z;
        const double d = (bz - cz) * (ax - cx) + (cx - bx) * (az - cz);
        if (std::fabs(d) < 1e-9) continue;
        const double l1 = ((bz - cz) * (x - cx) + (cx - bx) * (z - cz)) / d;
        const double l2 = ((cz - az) * (x - cx) + (ax - cx) * (z - cz)) / d;
        const double l3 = 1.0 - l1 - l2;
        const double e = -1e-6;
        if (l1 < e || l2 < e || l3 < e) continue;    // (x,z) outside this triangle
        hits.push_back(l1 * A.position.y + l2 * B.position.y + l3 * C.position.y);
    }
    std::sort(hits.begin(), hits.end());
    return hits;
}

// True if NO up-facing surface sits in the open height band (yLo,yHi) at (x,z) —
// i.e. clear air/sky between a low road and a high deck over the same footprint.
inline bool hasClearSpanAt(const RenderMesh& m, double x, double z, double yLo,
                           double yHi) {
    for (double h : surfaceHitsAt(m, x, z))
        if (h > yLo && h < yHi) return false;
    return true;
}

}  // namespace mesh_invariants
