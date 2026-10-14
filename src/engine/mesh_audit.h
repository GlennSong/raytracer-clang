#ifndef RAYTRACER_ENGINE_MESH_AUDIT_H
#define RAYTRACER_ENGINE_MESH_AUDIT_H

#include "mesh_builder.h"
#include "../renderer/renderer.h"
#include "../rt_math.h"

#include <cmath>
#include <cstdint>
#include <cstdio>

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
    Vec3 degenWhere{0, 0, 0}, degenA, degenB, degenC;   // the first degenerate one's corners
    void add(const MeshAudit& o) {
        if (degenerate == 0 && o.degenerate > 0) { degenWhere = o.degenWhere; degenA = o.degenA; degenB = o.degenB; degenC = o.degenC; }
        if (inward == 0 && o.inward > 0) where = o.where;
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
        if (cs < -0.3 && r.inward++ == 0) r.where = (a.position + b.position + c.position) / 3.0;
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
    if (a.inward > 0) std::printf("    [mesh audit] %-40s first inward at %.2f %.2f %.2f\n", "", a.where.x, a.where.y, a.where.z);
}

}  // namespace engine

#endif
