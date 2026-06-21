#include "road_mesh.h"

#include "../../mesh_builder.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace engine {

RenderMesh buildRoadMesh(const RoadGraph& g, const RoadMeshParams& p) {
    RenderMesh mesh;
    const int nNodes = static_cast<int>(g.nodes.size());
    const int nEdges = static_cast<int>(g.edges.size());

    auto height = [&](double x, double z) {
        return (p.heightAt ? p.heightAt(x, z) : 0.0) + p.lift;
    };
    auto to3d = [&](const Vec2& v) { return Vec3(v.x, height(v.x, v.y), v.y); };

    // Road geometry lies flat-ish on the terrain, so every tri faces up; emitTri
    // winds each one clockwise-front (the engine convention) regardless of the
    // order the strip/junction code happens to pass its corners in.
    auto addTri = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
        MeshBuilder::emitTri(mesh, a, b, c, Vec3(0, 1, 0), p.color);
    };
    // A flat strip between two cross-sections, segmented + draped on the terrain.
    auto addStrip = [&](const Vec2& a0, const Vec2& a1, const Vec2& b0,
                        const Vec2& b1) {
        double len = std::max((b0 - a0).length(), (b1 - a1).length());
        int segs = std::max(1, static_cast<int>(len / 4.0));   // follow terrain curvature
        Vec2 prev0 = a0, prev1 = a1;
        for (int s = 1; s <= segs; ++s) {
            double t = static_cast<double>(s) / segs;
            Vec2 c0 = lerp(a0, b0, t), c1 = lerp(a1, b1, t);
            addTri(to3d(prev0), to3d(prev1), to3d(c1));
            addTri(to3d(prev0), to3d(c1), to3d(c0));
            prev0 = c0; prev1 = c1;
        }
    };

    // Incident edges per node.
    std::vector<std::vector<int>> inc(nNodes);
    for (int e = 0; e < nEdges; ++e) {
        inc[g.edges[e].a].push_back(e);
        inc[g.edges[e].b].push_back(e);
    }

    // trim[e][0] at endpoint a, trim[e][1] at endpoint b: how far the ribbon is
    // pulled back from each node so it stops at the junction edge.
    std::vector<std::array<double, 2>> trim(nEdges, {0.0, 0.0});
    auto edgeLen = [&](int e) {
        return (g.nodes[g.edges[e].b].pos - g.nodes[g.edges[e].a].pos).length();
    };

    // --- Junctions: compute trims + build the pad, per node (degree >= 3) ------
    struct Arm {
        int edge; double ang; Vec2 d; double w; double s;   // s = setback at this node
    };
    for (int v = 0; v < nNodes; ++v) {
        if (static_cast<int>(inc[v].size()) < 3) continue;
        Vec2 V = g.nodes[v].pos;

        std::vector<Arm> arms;
        for (int e : inc[v]) {
            int other = (g.edges[e].a == v) ? g.edges[e].b : g.edges[e].a;
            Vec2 d = normalize(g.nodes[other].pos - V);
            arms.push_back({e, std::atan2(d.y, d.x), d, g.edges[e].width * 0.5, p.minSetback});
        }
        std::sort(arms.begin(), arms.end(),
                  [](const Arm& a, const Arm& b) { return a.ang < b.ang; });

        // Each adjacent pair of arms shares a curb corner = where arm A's left
        // side-line meets arm B's right side-line. The setback along each arm is
        // pushed out to clear that corner.
        int m = static_cast<int>(arms.size());
        for (int k = 0; k < m; ++k) {
            Arm& A = arms[k];
            Arm& B = arms[(k + 1) % m];
            Vec2 O1 = V + perp(A.d) * A.w;     // A's left (CCW) edge origin
            Vec2 O2 = V - perp(B.d) * B.w;     // B's right (CW) edge origin
            Vec2 r = O2 - O1;
            double denom = cross(A.d, B.d);
            double sA, sB;
            if (std::fabs(denom) < 1e-6) {     // near-parallel arms
                sA = A.w + B.w; sB = A.w + B.w;
            } else {
                sA = cross(r, B.d) / denom;
                sB = -cross(r, A.d) / denom;
            }
            A.s = std::max(A.s, sA);
            B.s = std::max(B.s, sB);
        }

        // Clamp + record the trims, and lay out the pad ring.
        std::vector<Vec2> ring;   // CCW: each arm contributes [right point, left point]
        for (Arm& a : arms) {
            double maxS = edgeLen(a.edge) * 0.45;
            a.s = std::min(std::max(a.s, p.minSetback), std::max(p.minSetback, maxS));
            int end = (g.edges[a.edge].a == v) ? 0 : 1;
            trim[a.edge][end] = a.s;
            Vec2 base = V + a.d * a.s;
            ring.push_back(base - perp(a.d) * a.w);   // right
            ring.push_back(base + perp(a.d) * a.w);   // left
        }
        // Fan-triangulate the pad from the node centre.
        Vec3 c = to3d(V);
        int rn = static_cast<int>(ring.size());
        for (int k = 0; k < rn; ++k)
            addTri(c, to3d(ring[k]), to3d(ring[(k + 1) % rn]));
    }

    // --- Ribbons: the trimmed span between junctions --------------------------
    for (int e = 0; e < nEdges; ++e) {
        Vec2 A = g.nodes[g.edges[e].a].pos, B = g.nodes[g.edges[e].b].pos;
        Vec2 ab = B - A;
        double len = ab.length();
        if (len < 1e-3) continue;
        Vec2 d = ab / len;
        double sa = trim[e][0], sb = trim[e][1];
        if (sa + sb >= len - 0.5) continue;        // swallowed entirely by junctions
        Vec2 start = A + d * sa, end = B - d * sb;
        Vec2 nrm = perp(d) * (g.edges[e].width * 0.5);
        addStrip(start - nrm, start + nrm, end - nrm, end + nrm);
    }

    return mesh;
}

}  // namespace engine
