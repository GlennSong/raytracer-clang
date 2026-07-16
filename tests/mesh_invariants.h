#pragma once
// Jolt-free geometric invariants for procedural meshes — the acceptance harness
// for the road/junction weld (road-unification-plan "Testing the viability of
// each join"). A weld regression (degenerate triangle, hole, flipped face) is
// otherwise invisible until seen by eye; these assert it mechanically over the
// output of buildRoadNetMesh / weldSolid for a matrix of join fixtures.

#include "../src/renderer/renderer.h"
#include "../src/rt_math.h"
#include <cmath>
#include <cstdint>

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

}  // namespace mesh_invariants
