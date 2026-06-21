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
    auto ground = [&](double x, double z) {
        return p.heightAt ? p.heightAt(x, z) : 0.0;          // terrain, no lift
    };
    auto to3d = [&](const Vec2& v) { return Vec3(v.x, height(v.x, v.y), v.y); };

    // Road geometry lies flat-ish on the terrain, so every tri faces up; emitTri
    // winds each one clockwise-front (the engine convention) regardless of the
    // order the strip/junction code happens to pass its corners in.
    auto addTri = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
        MeshBuilder::emitTri(mesh, a, b, c, Vec3(0, 1, 0), p.color);
    };

    // A raised sidewalk skirt running along a curb line P0->P1, with `outN` the
    // unit direction away from the road. Per sub-segment it emits the curb lip
    // (vertical face toward the street), the slab top (raised curbHeight above the
    // road), and an outer face dropping back to the ground — so the kerb reads as
    // a real raised sidewalk, not a painted stripe. No-op when disabled.
    auto curbBand = [&](const Vec2& P0, const Vec2& P1, const Vec2& outN) {
        if (p.sidewalkWidth <= 0.0) return;
        double len = (P1 - P0).length();
        if (len < 1e-3) return;
        int segs = std::max(1, static_cast<int>(len / 4.0));
        const Vec3 nUp(0, 1, 0);
        const Vec3 nIn(-outN.x, 0, -outN.y);    // curb lip faces the carriageway
        const Vec3 nOut(outN.x, 0, outN.y);     // outer face faces away
        for (int s = 1; s <= segs; ++s) {
            Vec2 q0 = lerp(P0, P1, static_cast<double>(s - 1) / segs);
            Vec2 q1 = lerp(P0, P1, static_cast<double>(s) / segs);
            Vec2 o0 = q0 + outN * p.sidewalkWidth, o1 = q1 + outN * p.sidewalkWidth;
            double r0 = height(q0.x, q0.y), r1 = height(q1.x, q1.y);   // road surface
            double t0 = r0 + p.curbHeight, t1 = r1 + p.curbHeight;     // slab top
            double g0 = std::min(ground(o0.x, o0.y), t0 - 0.01);       // outer foot
            double g1 = std::min(ground(o1.x, o1.y), t1 - 0.01);
            Vec3 a0(q0.x, r0, q0.y), a1(q1.x, r1, q1.y);   // curb base (at road)
            Vec3 b0(q0.x, t0, q0.y), b1(q1.x, t1, q1.y);   // curb top / slab inner
            Vec3 c0(o0.x, t0, o0.y), c1(o1.x, t1, o1.y);   // slab outer
            Vec3 d0(o0.x, g0, o0.y), d1(o1.x, g1, o1.y);   // outer foot
            MeshBuilder::emitQuad(mesh, a0, a1, b1, b0, nIn, p.curbColor);
            MeshBuilder::emitQuad(mesh, b0, b1, c1, c0, nUp, p.sidewalkColor);
            MeshBuilder::emitQuad(mesh, c0, c1, d1, d0, nOut, p.curbColor);
        }
    };

    // A painted lane stripe parallel to a ribbon: a thin strip at perpendicular
    // `offset` from the centreline, raised just above the asphalt. `dashed` lays
    // it as a dash pattern (lane dividers); solid otherwise (edge/centre lines).
    // `d`/`n` are the ribbon's unit direction / left-normal.
    auto laneStrip = [&](const Vec2& s, const Vec2& e, const Vec2& d, const Vec2& n,
                         double offset, const Vec3& col, bool dashed) {
        Vec2 a = s + n * offset, b = e + n * offset;
        double len = (b - a).length();
        if (len < 1e-3) return;
        double hw = p.markWidth * 0.5;
        auto P = [&](const Vec2& xz) {
            return Vec3(xz.x, height(xz.x, xz.y) + p.markLift, xz.y);
        };
        auto quad = [&](double t0, double t1) {
            Vec2 c0 = a + d * t0, c1 = a + d * t1;
            MeshBuilder::emitQuad(mesh, P(c0 - n * hw), P(c1 - n * hw),
                                  P(c1 + n * hw), P(c0 + n * hw), Vec3(0, 1, 0), col);
        };
        if (!dashed) {
            for (double t = 0; t < len; t += 3.0) quad(t, std::min(t + 3.0, len));
        } else {
            double period = p.dashLength + p.dashGap;
            for (double t = 0; t < len; t += period)
                quad(t, std::min(t + p.dashLength, len));
        }
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

        // A many-armed hub pushes every arm out to the plaza radius, so the pad
        // fills a clean circular plaza rather than a cramped fan of corners.
        double floorS = (m >= p.plazaMinArms && p.plazaRadius > 0.0)
                            ? std::max(p.minSetback, p.plazaRadius)
                            : p.minSetback;

        // Clamp + record the trims, and lay out the pad ring.
        std::vector<Vec2> ring;   // CCW: each arm contributes [right point, left point]
        for (Arm& a : arms) {
            double maxS = edgeLen(a.edge) * 0.45;
            a.s = std::min(std::max(a.s, floorS), std::max(floorS, maxS));
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

        // Sidewalk corners: the pad boundary between consecutive arms (arm k's
        // left point round to arm k+1's right point) is a kerb corner, not a road
        // mouth — skirt it so the sidewalk turns the corner instead of gapping.
        for (int k = 0; k < m; ++k) {
            const Vec2& Lk = ring[2 * k + 1];                  // arm k, left
            const Vec2& Rn = ring[2 * ((k + 1) % m)];          // arm k+1, right
            if ((Rn - Lk).length() < 1e-3) continue;
            Vec2 oN = normalize(perp(Rn - Lk));
            if (dot(oN, (Lk + Rn) * 0.5 - V) < 0) oN = oN * -1.0;   // face outward
            curbBand(Lk, Rn, oN);
        }
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
        Vec2 pu = perp(d);                              // unit edge normal
        Vec2 nrm = pu * (g.edges[e].width * 0.5);
        addStrip(start - nrm, start + nrm, end - nrm, end + nrm);
        // Sidewalk skirts down both verges (outward = away from the carriageway).
        curbBand(start + nrm, end + nrm, pu);
        curbBand(start - nrm, end - nrm, pu * -1.0);

        // Lane markings on the carriageway: solid edge lines, a double-yellow
        // centreline between opposing directions, dashed white lane dividers. The
        // lane count comes from the road width, so arterials read as multi-lane.
        if (p.laneMarkings) {
            double hw2 = g.edges[e].width * 0.5;
            // Equal lanes each side of a two-way centreline, so there is always a
            // centre and the dividers mirror.
            int perSide = std::max(1, static_cast<int>(std::lround(hw2 / p.laneWidth)));
            double laneW = hw2 / perSide, inset = p.markWidth * 1.5;
            // Double-yellow centreline between the opposing directions.
            laneStrip(start, end, d, pu,  p.markWidth, p.centerColor, false);
            laneStrip(start, end, d, pu, -p.markWidth, p.centerColor, false);
            // Dashed white lane dividers, mirrored each side.
            for (int i = 1; i < perSide; ++i) {
                laneStrip(start, end, d, pu,  i * laneW, p.laneColor, true);
                laneStrip(start, end, d, pu, -i * laneW, p.laneColor, true);
            }
            // Solid white edge lines just inside the kerb.
            laneStrip(start, end, d, pu,  hw2 - inset, p.laneColor, false);
            laneStrip(start, end, d, pu, -(hw2 - inset), p.laneColor, false);
        }
    }

    return mesh;
}

}  // namespace engine
