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

    // Turn the kerb around a corner at node V, from curb point P0 to P1 (outward =
    // away from V). Used at junction corners AND at simple bends, so adjacent
    // sidewalks join cleanly instead of leaving a notch where the kerbs meet.
    auto cornerBand = [&](const Vec2& V, const Vec2& P0, const Vec2& P1) {
        if ((P1 - P0).length() < 1e-3) return;
        Vec2 oN = normalize(perp(P1 - P0));
        if (dot(oN, (P0 + P1) * 0.5 - V) < 0) oN = oN * -1.0;
        curbBand(P0, P1, oN);
    };

    // Incident edges per node.
    std::vector<std::vector<int>> inc(nNodes);
    for (int e = 0; e < nEdges; ++e) {
        inc[g.edges[e].a].push_back(e);
        inc[g.edges[e].b].push_back(e);
    }

    // Flag the sharp degree-2 bends (hairpins): a turn this tight can't be a simple
    // bend — the widened ribbon would fold — so it's built as a turning pad below.
    std::vector<char> hairpin(nNodes, 0);
    if (p.hairpinDeflection > 0.0) {
        for (int v = 0; v < nNodes; ++v) {
            if (static_cast<int>(inc[v].size()) != 2) continue;
            Vec2 V = g.nodes[v].pos;
            auto dir = [&](int e) {
                int o = (g.edges[e].a == v) ? g.edges[e].b : g.edges[e].a;
                return normalize(g.nodes[o].pos - V);
            };
            Vec2 d0 = dir(inc[v][0]), d1 = dir(inc[v][1]);
            double between = std::acos(std::max(-1.0, std::min(1.0, dot(d0, d1))));
            if (3.14159265358979 - between > p.hairpinDeflection) hairpin[v] = 1;   // deflection too sharp
        }
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
        for (int k = 0; k < m; ++k)
            cornerBand(V, ring[2 * k + 1], ring[2 * ((k + 1) % m)]);
    }

    // --- Hairpins (sharp degree-2): a clean turning-head DISC. The junction pad
    // assumes arms spread around the node and fan-triangulates a ring; two
    // near-parallel hairpin arms make that fan wrap a ~340-degree triangle across the
    // back and fold over itself. A disc centred on the apex can't fold: pull both
    // legs back into it and fill it as a fan, so the tight inner curve is covered by
    // a turning head rather than a folded ribbon.
    for (int v = 0; v < nNodes; ++v) {
        if (!hairpin[v]) continue;
        Vec2 V = g.nodes[v].pos;
        double maxHalf = std::max(g.edges[inc[v][0]].width, g.edges[inc[v][1]].width) * 0.5;
        double tr = maxHalf;                                  // pull leg ends inside the disc
        for (int e : inc[v]) {
            int end = (g.edges[e].a == v) ? 0 : 1;
            trim[e][end] = std::max(trim[e][end], std::min(tr, edgeLen(e) * 0.45));
        }
        double R = maxHalf * 1.5 + p.sidewalkWidth;           // covers the trimmed leg corners
        Vec3 c = to3d(V);
        int segs = std::max(10, static_cast<int>(std::ceil(2.0 * 3.14159265 * R / 4.0)));
        Vec2 prev = V + Vec2(R, 0);
        for (int s = 1; s <= segs; ++s) {
            double ang = 2.0 * 3.14159265 * s / segs;
            Vec2 cur = V + Vec2(std::cos(ang) * R, std::sin(ang) * R);
            addTri(c, to3d(prev), to3d(cur));                 // CCW fan -> upward normal
            prev = cur;
        }
    }

    // --- Simple bends (degree-2): no pad, but the two ribbons still meet at an
    // angle (every chord-joint of a curved ring), so their kerbs leave a notch.
    // Fill the small carriageway gap and turn the sidewalk corner on each side so
    // the kerb runs continuous around the curve. The road runs straight through,
    // so there is no trim.
    for (int v = 0; v < nNodes; ++v) {
        if (static_cast<int>(inc[v].size()) != 2 || hairpin[v]) continue;   // hairpins padded above
        Vec2 V = g.nodes[v].pos;
        auto armDir = [&](int e) {
            int o = (g.edges[e].a == v) ? g.edges[e].b : g.edges[e].a;
            return normalize(g.nodes[o].pos - V);
        };
        int e0 = inc[v][0], e1 = inc[v][1];
        Vec2 d0 = armDir(e0), d1 = armDir(e1);
        double w0 = g.edges[e0].width * 0.5, w1 = g.edges[e1].width * 0.5;
        Vec2 l0 = V + perp(d0) * w0, r0 = V - perp(d0) * w0;   // e0 kerb points at V
        Vec2 l1 = V + perp(d1) * w1, r1 = V - perp(d1) * w1;   // e1 kerb points at V
        // The road continues, so e0's left pairs with e1's right (and vice versa).
        // Fan the carriageway notch from V to each pair, then turn the kerb across it.
        addTri(to3d(V), to3d(l0), to3d(r1));
        addTri(to3d(V), to3d(r0), to3d(l1));
        cornerBand(V, l0, r1);
        cornerBand(V, r0, l1);
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

RenderMesh strokeRibbon(const std::vector<Vec2>& pts, const std::vector<double>& halfW,
                        double y, const Vec3& color, bool closed) {
    RenderMesh mesh;
    const int n = static_cast<int>(pts.size());
    if (n < 2 || halfW.empty()) return mesh;

    auto W   = [&](int i) { return halfW[std::min<int>(static_cast<int>(halfW.size()) - 1,
                                                       std::max(0, i))]; };
    auto P3  = [&](const Vec2& v) { return Vec3(v.x, y, v.y); };
    auto tri = [&](const Vec2& a, const Vec2& b, const Vec2& c) {
        MeshBuilder::emitTri(mesh, P3(a), P3(b), P3(c), Vec3(0, 1, 0), color);
    };

    // Each segment is a trapezoid (start half-width -> end half-width). On the inside
    // of a bend these overlap, but they're coplanar fills facing the same way — not a
    // fold — so the union is exactly the stroked region.
    const int segCount = closed ? n : n - 1;
    for (int i = 0; i < segCount; ++i) {
        Vec2 A = pts[i], B = pts[(i + 1) % n];
        Vec2 ab = B - A; double L = ab.length();
        if (L < 1e-9) continue;
        Vec2 nrm = perp(ab / L);
        double wa = W(i), wb = W((i + 1) % n);
        Vec2 AL = A + nrm*wa, AR = A - nrm*wa, BL = B + nrm*wb, BR = B - nrm*wb;
        tri(AL, AR, BR); tri(AL, BR, BL);
    }

    // Round join at each vertex: fan the OUTER wedge of the bend between the two
    // segments' edges (sweep == the signed turn angle, so a 180-degree vertex gets a
    // semicircular cap). The inner side's trapezoids overlap to meet. To avoid a
    // sliver where the two trapezoids leave a hairline gap (and the tiny inner cusp at
    // a convex vertex), the inner pair of corners is bridged with a single bevel
    // triangle — cheap, and harmless where it overlaps.
    const int vbeg = closed ? 0 : 1, vend = closed ? n : n - 1;
    for (int i = vbeg; i < vend; ++i) {
        Vec2 P = pts[i];
        Vec2 a = P - pts[(i - 1 + n) % n], b = pts[(i + 1) % n] - P;
        double la = a.length(), lb = b.length();
        if (la < 1e-9 || lb < 1e-9) continue;
        a = a / la; b = b / lb;
        double theta = std::atan2(cross(a, b), dot(a, b));   // signed turn (-pi..pi)
        if (std::fabs(theta) < 1e-4) continue;               // straight: segments meet flush
        double w = W(i);
        double outer = theta > 0 ? -1.0 : 1.0;               // outer side sign
        Vec2 c0dir = perp(a) * outer;                        // dir to the outer corner
        double a0 = std::atan2(c0dir.y, c0dir.x);
        int segs = std::max(1, static_cast<int>(std::ceil(std::fabs(theta) / 0.30)));
        Vec2 prev = P + c0dir * w;
        for (int s = 1; s <= segs; ++s) {
            double ang = a0 + theta * (static_cast<double>(s) / segs);
            tri(P, prev, P + Vec2(std::cos(ang), std::sin(ang)) * w);
            prev = P + Vec2(std::cos(ang), std::sin(ang)) * w;
        }
        // Inner bevel: bridge the two inner corners so no hairline gap/cusp remains.
        tri(P, P - perp(a) * outer * w, P - perp(b) * outer * w);
    }
    return mesh;
}

}  // namespace engine
