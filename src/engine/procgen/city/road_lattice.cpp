#include "road_lattice.h"

#include "../../mesh_builder.h"      // MeshBuilder::emitLattice
#include "triangulate.h"           // junction pad ear-clip (roads-v2)
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <map>
#include <cstdio>
#include <cstdlib>

namespace engine {

namespace {

// One resampled station along the chain: where the ring sits and the local frame
// the profile is placed in.
struct Ring {
    Vec2 c;          // centreline position (world XZ)
    Vec2 fwd;        // unit tangent (XZ)
    Vec2 left;       // unit +lateral (XZ), = fwd rotated +90
    double yBase;    // deck-plane Y at the centreline
    double hw;       // half-width here (the deck flares with it)
    double cs;       // cross-slope (superelevation) here
    double s;        // arc length from the chain start
    double miter = 1.0;   // lateral scale at a bend: 1/cos(halfAngle), clamped
};

double lerp(double a, double b, double t) { return a + (b - a) * t; }

// Resample the spine at ~ringStep spacing, carrying yAbs / hw / crossSlope.
// Tangents are a central difference over the resampled centres, so the frame
// turns smoothly through a bend instead of faceting at each control point.
std::vector<Ring> sampleRings(const UnionSpine& spine, double ringStep,
                              const std::function<double(double, double)>& ground) {
    std::vector<Ring> rings;
    const std::size_t n = spine.points.size();
    if (n < 2) return rings;
    const bool hasY = spine.yAbs.size() == n;
    const bool hasW = spine.hw.size() == n;
    const bool hasC = spine.crossSlope.size() == n;

    // Per-segment arc length.
    std::vector<double> segLen(n - 1);
    double total = 0;
    for (std::size_t i = 0; i + 1 < n; ++i) {
        segLen[i] = (spine.points[i + 1] - spine.points[i]).length();
        total += segLen[i];
    }
    if (total < 1e-6) return rings;

    // Ring stations. A SPARSE spine (authored street graph, segments much longer
    // than the ring pitch) gets a ring at EVERY vertex — a kink between stations
    // would have its corner chopped by the spanning quad (the unmitered look) —
    // plus even fill between vertices. A DENSE spine (corridor decks/ramps
    // already sampled at ~ring pitch) keeps pure even spacing: its vertices are
    // sampling artifacts, not kinks, and re-phasing rings there shifts tuned
    // arc-length features (parapet gore gaps).
    std::vector<double> stations;
    const double avgSeg = total / (n - 1);
    if (avgSeg < ringStep * 1.5) {
        const int R = std::max(1, static_cast<int>(std::ceil(total / std::max(0.25, ringStep))));
        for (int k = 0; k <= R; ++k) stations.push_back(total * k / R);
    } else {
        double cum = 0;
        stations.push_back(0.0);
        for (std::size_t i = 0; i + 1 < n; ++i) {
            const int fill = std::max(1, static_cast<int>(std::ceil(segLen[i] / std::max(0.25, ringStep))));
            for (int k = 1; k <= fill; ++k) stations.push_back(cum + segLen[i] * k / fill);
            cum += segLen[i];
        }
    }
    for (double s : stations) {
        // locate the segment
        double acc = 0; std::size_t si = 0;
        while (si + 1 < segLen.size() && acc + segLen[si] < s) { acc += segLen[si]; ++si; }
        const double t = segLen[si] > 1e-9 ? (s - acc) / segLen[si] : 0.0;
        Ring rg;
        rg.c = spine.points[si] + (spine.points[si + 1] - spine.points[si]) * t;
        rg.hw = hasW ? lerp(spine.hw[si], spine.hw[si + 1], t) : spine.halfWidth;
        rg.cs = hasC ? lerp(spine.crossSlope[si], spine.crossSlope[si + 1], t) : 0.0;
        rg.yBase = hasY ? lerp(spine.yAbs[si], spine.yAbs[si + 1], t)
                        : (ground ? ground(rg.c.x, rg.c.y) : 0.0);
        rg.s = s;
        rings.push_back(rg);
    }
    // Central-difference tangents; left = tangent rotated +90 in XZ. At a bend
    // the central difference IS the miter direction, but offsetting by the raw
    // width along it NARROWS the ribbon by cos(halfAngle) — the unmitered
    // sidewalk. Scale lateral offsets by the miter length 1/cos(halfAngle),
    // clamped so acute kinks cannot spike.
    for (std::size_t i = 0; i < rings.size(); ++i) {
        const Vec2 prev = rings[i == 0 ? 0 : i - 1].c;
        const Vec2 next = rings[i + 1 < rings.size() ? i + 1 : i].c;
        Vec2 d = next - prev;
        if (d.lengthSquared() < 1e-12) d = Vec2(1, 0);
        rings[i].fwd = normalize(d);
        rings[i].left = Vec2(-rings[i].fwd.y, rings[i].fwd.x);
        // Miter only DRAPED street spines: authored-yAbs chains (freeway decks,
        // ramps) are smooth clothoids that never kink — and an acute artifact
        // vertex there would flare a parapet into the travel lane. Clamp at 2x
        // so even a sharp street corner widens, never spikes.
        if (!hasY) {
            Vec2 dSeg = next - rings[i].c;                   // local segment dir
            if (dSeg.lengthSquared() < 1e-12) dSeg = rings[i].c - prev;
            if (dSeg.lengthSquared() > 1e-12)
                rings[i].miter = std::min(2.0,
                    1.0 / std::max(0.5, (double)dot(rings[i].fwd, normalize(dSeg))));
        }
    }
    return rings;
}

}  // namespace

RenderMesh sweepRoadLattice(const UnionSpine& spine, const RoadProfile& profile,
                            const std::function<double(double, double)>& ground,
                            double ringStep,
                            const std::vector<GapWindow>* gaps,
                            std::vector<Vec3>* ring0Out,
                            std::vector<Vec3>* ringNOut) {
    RenderMesh mesh;
    const int P = static_cast<int>(profile.cols.size());
    if (P < 2) return mesh;
    std::vector<Ring> rings = sampleRings(spine, ringStep, ground);
    if (rings.size() < 2) return mesh;
    const int R = static_cast<int>(rings.size());

    std::vector<Vertex> verts(static_cast<std::size_t>(R) * P);
    for (int i = 0; i < R; ++i) {
        const Ring& rg = rings[i];
        const Vec3 c3(rg.c.x, 0, rg.c.y);
        const Vec3 left3(rg.left.x, 0, rg.left.y);
        // The banked deck up-normal: moving +1 lateral raises Y by cs, so the
        // surface normal tilts away from the high side.
        const Vec3 bankedUp = normalize(Vec3(0, 1, 0) - left3 * rg.cs);
        for (int j = 0; j < P; ++j) {
            const ProfileCol& col = profile.cols[j];
            const double lateral = (col.edgeFrac * rg.hw + col.absOffset) * rg.miter;
            const double deckY = rg.yBase + lateral * rg.cs;    // banked deck plane
            Vertex& v = verts[i * P + j];
            v.position = c3 + left3 * lateral;
            v.position.y = deckY + col.height;
            // Cross-section normal (cnLat along +left, cnVert along up), banked.
            Vec3 nrm = left3 * col.cnLat + bankedUp * col.cnVert;
            v.normal = nrm.lengthSquared() > 1e-12 ? normalize(nrm) : Vec3(0, 1, 0);
            v.tangent = Vec3(rg.fwd.x, 0, rg.fwd.y);
            v.u = col.mu;
            v.v = static_cast<float>(rg.s);
            v.color = col.color;
        }
    }

    // Expose the end rings (all columns) so a junction patch can reuse the body's
    // own mouth vertices — the body and the patch then share the seam exactly.
    if (ring0Out) { ring0Out->clear();
        for (int j = 0; j < P; ++j) ring0Out->push_back(verts[j].position); }
    if (ringNOut) { ringNOut->clear();
        for (int j = 0; j < P; ++j) ringNOut->push_back(verts[(R - 1) * P + j].position); }

    // Which rings survive the gap windows. A gapped profile (an edge parapet
    // over a ramp gore) is emitted as several lattices, one per active run, so
    // the opening is a clean break with no zero-area geometry across it.
    auto ringActive = [&](int i) {
        if (!gaps) return true;
        for (const GapWindow& g : *gaps)
            if (rings[i].s > g.s0 && rings[i].s < g.s1) return false;
        return true;
    };
    int runStart = -1;
    auto flushRun = [&](int endExclusive) {
        if (runStart < 0 || endExclusive - runStart < 2) { runStart = -1; return; }
        MeshBuilder::emitLattice(
            mesh, { endExclusive - runStart, P, verts.data() + static_cast<std::size_t>(runStart) * P });
        runStart = -1;
    };
    for (int i = 0; i < R; ++i) {
        if (ringActive(i)) { if (runStart < 0) runStart = i; }
        else flushRun(i);
    }
    flushRun(R);
    return mesh;
}

RoadProfile freewayDeckProfile(int lanesPerSide, double laneWidth) {
    (void)laneWidth;   // width comes from the spine's hw; lanes only set column count
    RoadProfile prof;
    const int cols = std::max(2, 2 * std::max(1, lanesPerSide) + 1);
    const Vec3 asphalt(0.10, 0.10, 0.11);
    for (int j = 0; j < cols; ++j) {
        const double frac = -1.0 + 2.0 * j / (cols - 1);    // -1 (right) .. +1 (left)
        ProfileCol col;
        col.edgeFrac = frac;
        col.cnVert = 1.0;                                    // drivable, faces up
        // Carriageway paint coord: lateral -hw..+hw -> mu 1..3, centre = 2.
        col.mu = static_cast<float>(2.0 + frac);
        col.color = asphalt;
        prof.cols.push_back(col);
    }
    return prof;
}

namespace {
const Vec3 kConcrete(0.45, 0.46, 0.48);
ProfileCol pc(double edgeFrac, double absOffset, double height,
              double cnLat, double cnVert, float mu, const Vec3& col) {
    ProfileCol c;
    c.edgeFrac = edgeFrac; c.absOffset = absOffset; c.height = height;
    c.cnLat = cnLat; c.cnVert = cnVert; c.mu = mu; c.color = col;
    return c;
}
}  // namespace

RoadProfile freewayUndersideProfile(double thickness) {
    // Down the near fascia, across the soffit, up the far fascia. Crease columns
    // are beveled (not duplicated), so there is no zero-area connector band; a
    // viaduct's underside edges don't need a razor crease.
    RoadProfile p;
    p.cols = {
        pc(+1, 0, 0, +1.0, 0.0, 0.5f, kConcrete),                 // near fascia top
        pc(+1, 0, -thickness, +0.6, -0.8, 0.5f, kConcrete),       // near fascia foot
        pc(-1, 0, -thickness, -0.6, -0.8, 0.5f, kConcrete),       // far soffit corner
        pc(-1, 0, 0, -1.0, 0.0, 0.5f, kConcrete),                 // far fascia top
    };
    return p;
}

RoadProfile parapetProfile(double side, double height, double thickness) {
    // A wall standing at the `side` verge (+1 left / -1 right), `thickness`
    // inboard. Inner face toward the road, top, outer face toward the edge.
    RoadProfile p;
    p.cols = {
        pc(side, -side * thickness, 0, -side, 0.0, 0.5f, kConcrete),        // inner base
        pc(side, -side * thickness, height, -side * 0.7, 0.7, 0.5f, kConcrete),  // inner top
        pc(side, 0, height, side * 0.7, 0.7, 0.5f, kConcrete),              // outer top
        pc(side, 0, 0, side, 0.0, 0.5f, kConcrete),                        // outer base
    };
    return p;
}

RoadProfile medianProfile(double halfWidth, double height) {
    RoadProfile p;
    p.cols = {
        pc(0, +halfWidth, 0, +1.0, 0.0, 0.5f, kConcrete),          // left face base
        pc(0, +halfWidth, height, +0.7, 0.7, 0.5f, kConcrete),     // left top
        pc(0, -halfWidth, height, -0.7, 0.7, 0.5f, kConcrete),     // right top
        pc(0, -halfWidth, 0, -1.0, 0.0, 0.5f, kConcrete),          // right face base
    };
    return p;
}

RoadProfile streetProfile(int lanesPerSide, double sidewalkWidth, double curbHeight) {
    RoadProfile p;
    const Vec3 asphalt(0.10, 0.10, 0.11), walk(0.62, 0.62, 0.60);
    // Left: sidewalk (raised) -> curb top -> down the curb face to the carriageway.
    p.cols.push_back(pc(-1, -sidewalkWidth, curbHeight, 0.0, 1.0, -0.6f, walk));  // sidewalk outer
    p.cols.push_back(pc(-1, 0, curbHeight, -0.4, 0.9, -0.05f, walk));             // curb top (crease)
    const int cw = std::max(3, 2 * std::max(1, lanesPerSide) + 1);
    for (int j = 0; j < cw; ++j) {
        const double f = -1.0 + 2.0 * j / (cw - 1);
        // curb-face crease at the verges: tilt the edge column's normal a little
        // so the 0.15 m curb reads as a step rather than flat asphalt.
        const double nl = (j == 0) ? 0.4 : (j == cw - 1 ? -0.4 : 0.0);
        p.cols.push_back(pc(f, 0, 0, nl, j == 0 || j == cw - 1 ? 0.9 : 1.0,
                            static_cast<float>(2.0 + f), asphalt));
    }
    p.cols.push_back(pc(+1, 0, curbHeight, 0.4, 0.9, -0.05f, walk));              // curb top (crease)
    p.cols.push_back(pc(+1, +sidewalkWidth, curbHeight, 0.0, 1.0, -0.6f, walk));  // sidewalk outer
    return p;
}

RoadProfile carriagewayProfile(int lanesPerSide) {
    RoadProfile p;
    const Vec3 asphalt(0.10, 0.10, 0.11);
    const int cw = std::max(3, 2 * std::max(1, lanesPerSide) + 1);
    for (int j = 0; j < cw; ++j) {
        const double f = -1.0 + 2.0 * j / (cw - 1);
        // curb-face crease at the verges: tilt the edge column's normal a little
        // so the curb band beside it reads as a step rather than flat asphalt.
        const double nl = (j == 0) ? 0.4 : (j == cw - 1 ? -0.4 : 0.0);
        p.cols.push_back(pc(f, 0, 0, nl, j == 0 || j == cw - 1 ? 0.9 : 1.0,
                            static_cast<float>(2.0 + f), asphalt));
    }
    return p;
}

RenderMesh sweepFreewayDeck(const UnionSpine& spine,
                            const std::function<double(double, double)>& ground,
                            double ringStep, double deckThickness,
                            const std::vector<GapWindow>* gaps) {
    RenderMesh mesh;
    // Drivable deck + structural underside run the full length.
    MeshBuilder::append(mesh, sweepRoadLattice(spine, freewayDeckProfile(), ground, ringStep));
    MeshBuilder::append(mesh, sweepRoadLattice(spine, freewayUndersideProfile(deckThickness),
                                               ground, ringStep));
    // Edge parapets gap over each ramp gore so a car can merge.
    MeshBuilder::append(mesh, sweepRoadLattice(spine, parapetProfile(+1), ground, ringStep, gaps));
    MeshBuilder::append(mesh, sweepRoadLattice(spine, parapetProfile(-1), ground, ringStep, gaps));
    // Median wall runs the full length (ramps meet the edges, not the centre).
    MeshBuilder::append(mesh, sweepRoadLattice(spine, medianProfile(), ground, ringStep));
    return mesh;
}

RenderMesh sweepRampDeck(const UnionSpine& spine,
                         const std::function<double(double, double)>& ground,
                         double ringStep, double deckThickness) {
    RenderMesh mesh;
    MeshBuilder::append(mesh, sweepRoadLattice(spine, freewayDeckProfile(1), ground, ringStep));
    MeshBuilder::append(mesh, sweepRoadLattice(spine, freewayUndersideProfile(deckThickness),
                                               ground, ringStep));
    MeshBuilder::append(mesh, sweepRoadLattice(spine, parapetProfile(+1), ground, ringStep));
    MeshBuilder::append(mesh, sweepRoadLattice(spine, parapetProfile(-1), ground, ringStep));
    return mesh;
}

RenderMesh sweepCorridor(const UnionSpine& deckSpine,
                         const std::vector<UnionSpine>& rampSpines,
                         const CorridorLatticeParams& p) {
    RenderMesh mesh;
    MeshBuilder::append(mesh, sweepFreewayDeck(deckSpine, p.ground, p.ringStep,
                                               p.deckThickness, &p.deckGaps));
    for (const UnionSpine& r : rampSpines)
        MeshBuilder::append(mesh, sweepRampDeck(r, p.ground, p.ringStep, p.deckThickness));

    // Piers under the elevated spans (shared helper — the street lattice uses
    // the same placement for authored viaducts and layered bridges).
    MeshBuilder::append(mesh, latticeChainPiers(deckSpine, p.ground, p.deckThickness,
                                                p.pierBasesOut));
    return mesh;
}

RenderMesh latticeChainPiers(const UnionSpine& spine,
                             const std::function<double(double, double)>& ground,
                             double deckThickness, std::vector<Vec2>* pierBasesOut,
                             const std::function<bool(const Vec2&)>& keepOut) {
    RenderMesh mesh;
    if (spine.yAbs.size() != spine.points.size()) return mesh;
    const std::vector<Vec2>& cl = spine.points;
    const std::vector<double>& dy = spine.yAbs;
    std::vector<int> at;
    double since = 1e9;
    for (std::size_t i = 0; i < cl.size(); ++i) {
        const double g = ground ? ground(cl[i].x, cl[i].y) : 0.0;
        const double clr = dy[i] - g;
        const double seg = i == 0 ? 0.0 : (cl[i] - cl[i - 1]).length();
        since += seg;
        if (clr > 2.0 && since >= 24.0) {
            if (keepOut && keepOut(cl[i])) continue;   // no column in a road below
            at.push_back(static_cast<int>(i));
            since = 0.0;
        }
    }
    if (!at.empty()) {
        MeshBuilder::append(mesh, bridgePiers(cl, dy, at, 2.0, 1.4, deckThickness,
                                              Vec3(0.45, 0.46, 0.48), ground));
        if (pierBasesOut)
            for (int i : at) pierBasesOut->push_back(cl[i]);
    }
    return mesh;
}

RenderMesh coonsPatch(const std::vector<Vec3>& bottom, const std::vector<Vec3>& right,
                      const std::vector<Vec3>& top, const std::vector<Vec3>& left,
                      float mu, const Vec3& color) {
    RenderMesh mesh;
    const int nu = static_cast<int>(bottom.size()) - 1;   // u-direction spans
    const int nv = static_cast<int>(left.size()) - 1;     // v-direction spans
    if (nu < 1 || nv < 1 || static_cast<int>(top.size()) != nu + 1 ||
        static_cast<int>(right.size()) != nv + 1)
        return mesh;

    const Vec3 P00 = bottom.front(), P10 = bottom.back();
    const Vec3 P01 = top.front(),    P11 = top.back();

    const int R = nv + 1, P = nu + 1;
    std::vector<Vec3> pos(static_cast<std::size_t>(R) * P);
    for (int j = 0; j <= nv; ++j) {
        const double v = static_cast<double>(j) / nv;
        for (int i = 0; i <= nu; ++i) {
            const double u = static_cast<double>(i) / nu;
            // Transfinite bilinear (Coons) interpolation: the two ruled surfaces
            // between opposite sides, minus the bilinear surface of the corners.
            const Vec3  ruledU = bottom[i] * (1 - v) + top[i] * v;
            const Vec3 ruledV = left[j] * (1 - u) + right[j] * u;
            const Vec3 bilin = P00 * ((1 - u) * (1 - v)) + P10 * (u * (1 - v)) +
                               P01 * ((1 - u) * v) + P11 * (u * v);
            pos[j * P + i] = ruledU + ruledV - bilin;
        }
    }

    // Per-vertex normals from the grid tangents, so the pad reads as the (gently
    // sloped) driving surface it is — the drive probe keeps up-facing faces.
    std::vector<Vertex> verts(static_cast<std::size_t>(R) * P);
    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            const Vec3& c = pos[j * P + i];
            const Vec3 du = pos[j * P + std::min(i + 1, nu)] - pos[j * P + std::max(i - 1, 0)];
            const Vec3 dv = pos[std::min(j + 1, nv) * P + i] - pos[std::max(j - 1, 0) * P + i];
            Vec3 n = cross(dv, du);
            if (n.y < 0) n = n * -1.0;                    // keep it up-facing
            Vertex& vv = verts[j * P + i];
            vv.position = c;
            vv.normal = n.lengthSquared() > 1e-12 ? normalize(n) : Vec3(0, 1, 0);
            vv.tangent = du.lengthSquared() > 1e-12 ? normalize(du) : Vec3(1, 0, 0);
            vv.u = mu;
            vv.v = 0.0f;
            vv.color = color;
        }
    }
    MeshBuilder::emitLattice(mesh, { R, P, verts.data() });
    return mesh;
}

// ---------------------------------------------------------------------------
// Junction fill (roads-v2 Part 2, step 4) — ONE path for every degree.
// S4: kerb-return FILLET ARCS between adjacent arms, a GRID interior with
// Laplace-smoothed heights, and an annulus stitch that touches EVERY boundary
// vertex (exact K — the pad can never T-joint against a body's end ring).
// Fallback for pads too small (or too odd) to grid: plain boundary ear-clip.

namespace {

// The S1 fill, kept as the small-pad fallback: ear-clip the boundary loop and
// lift by the boundary heights (earcut may prune collinear midpoints — that is
// exactly why it is only the fallback for pads with no room for a grid).
RenderMesh junctionPadEarcut(const std::vector<Vec3>& loop, float mu, const Vec3& color) {
    RenderMesh mesh;
    Poly2 outer;
    outer.reserve(loop.size());
    for (const Vec3& p : loop) outer.push_back(Vec2(p.x, p.z));
    const std::vector<std::array<Vec2, 3>> tris = triangulateWithHoles(outer, {});
    auto qkey = [](const Vec2& q) {
        return (static_cast<long long>(std::llround(q.x * 1024.0)) << 32) ^
               (static_cast<long long>(std::llround(q.y * 1024.0)) & 0xffffffffLL);
    };
    std::unordered_map<long long, double> heightOf;
    heightOf.reserve(loop.size() * 2);
    for (const Vec3& p : loop) heightOf[qkey(Vec2(p.x, p.z))] = p.y;
    auto lift = [&](const Vec2& q) {
        auto it = heightOf.find(qkey(q));
        return Vec3(q.x, it != heightOf.end() ? it->second : 0.0, q.y);
    };
    for (const std::array<Vec2, 3>& t : tris) {
        const Vec3 a = lift(t[0]), b = lift(t[1]), c = lift(t[2]);
        Vec3 n = cross(c - a, b - a);
        if (n.lengthSquared() < 1e-12) continue;
        if (n.y < 0) n = n * -1.0;                    // drivable pad faces up
        MeshBuilder::emitTriUV(mesh, a, b, c, normalize(n), color,
                               mu, 0.0f, mu, 0.0f, mu, 0.0f);
    }
    if (std::getenv("RT_PAD_DEBUG")) {
        double polyA = 0, triA = 0;
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const Vec3& p = loop[i];
            const Vec3& q = loop[(i + 1) % loop.size()];
            polyA += p.x * q.z - q.x * p.z;
        }
        for (const std::array<Vec2, 3>& t : tris)
            triA += std::fabs((t[1].x - t[0].x) * (t[2].y - t[0].y) -
                              (t[2].x - t[0].x) * (t[1].y - t[0].y));
        std::fprintf(stderr, "[pad] earcut: loop=%zu tris=%zu polyArea=%.1f triArea=%.1f\n",
                     loop.size(), tris.size(), std::fabs(polyA) * 0.5, triA * 0.5);
    }
    return mesh;
}

}  // namespace

RenderMesh junctionPatch(std::vector<JunctionArm> arms, float mu, const Vec3& color,
                         std::vector<Vec3>* footprintOut) {
    RenderMesh mesh;
    const int N = static_cast<int>(arms.size());
    if (N <= 2) return mesh;                          // body runs straight through
    for (const JunctionArm& arm : arms)
        if (arm.mouth.size() < 2) return mesh;

    // Order arms CCW around the centroid of their mouth CENTRES — not by the
    // outward dir bearing: a compound pad (two merged junctions) holds arms
    // from different member nodes, and two parallel outer arms would tie on
    // bearing while their positions order them unambiguously.
    Vec2 ctr(0, 0);
    auto mouthCentre = [](const JunctionArm& a) {
        Vec3 s(0, 0, 0);
        for (const Vec3& p : a.mouth) s = s + p;
        s = s * (1.0 / static_cast<double>(a.mouth.size()));
        return Vec2(s.x, s.z);
    };
    for (const JunctionArm& a : arms) ctr = ctr + mouthCentre(a);
    ctr = ctr * (1.0 / N);
    if (std::getenv("RT_PAD_DEBUG"))
        for (const JunctionArm& a : arms)
            std::fprintf(stderr,
                         "[pad] arm dir=(%.2f,%.2f) mouth (%.1f,%.1f)->(%.1f,%.1f) K=%zu\n",
                         a.dir.x, a.dir.y, a.mouth.front().x, a.mouth.front().z,
                         a.mouth.back().x, a.mouth.back().z, a.mouth.size());
    std::sort(arms.begin(), arms.end(), [&](const JunctionArm& a, const JunctionArm& b) {
        const Vec2 ca = mouthCentre(a) - ctr, cb = mouthCentre(b) - ctr;
        return std::atan2(ca.y, ca.x) < std::atan2(cb.y, cb.x);
    });

    // Closed boundary loop, CCW: each arm's mouth right->left (back->front),
    // then the KERB-RETURN FILLET to the next arm — a quadratic arc from arm
    // i's left verge to arm i+1's right verge, bulging toward the verge-line
    // intersection (the rounded concave corner a real kerb return cuts),
    // instead of S1's snapped midpoint. Mouth vertices are tagged: they are
    // the bodies' own end-ring points and every one must survive to the fill.
    std::vector<Vec3> loop;
    std::vector<char> isMouth;
    std::vector<int> armOf;               // sorted-arm index per vertex (-1 = arc)
    auto push = [&](const Vec3& p, int arm) {
        const Vec2 p2(p.x, p.z);
        if (!loop.empty() &&
            (Vec2(loop.back().x, loop.back().z) - p2).lengthSquared() < 1e-10) {
            if (arm >= 0) { isMouth.back() = 1; armOf.back() = arm; }
            // Two arms sharing a corner point at DIFFERENT heights is a
            // contradictory input (real chain profiles reconcile at nodes);
            // the pad owns the seam and splits the difference.
            loop.back().y = (loop.back().y + p.y) * 0.5;
            return;
        }
        loop.push_back(p);
        isMouth.push_back(arm >= 0);
        armOf.push_back(arm);
    };
    for (int i = 0; i < N; ++i) {
        const JunctionArm& A = arms[i];
        const JunctionArm& B = arms[(i + 1) % N];
        for (std::size_t j = A.mouth.size(); j-- > 0;) push(A.mouth[j], i);
        const Vec3 a3 = A.mouth.front(), b3 = B.mouth.back();
        const Vec2 a2(a3.x, a3.z), b2(b3.x, b3.z);
        const double chord = (b2 - a2).length();
        if (chord < 0.6) continue;            // touching mouths: shared corner
        // Control point: where the two verge lines (running along each arm's
        // direction) meet. Near-parallel arms have no stable intersection —
        // fall back to the straight chord.
        Vec2 c2 = (a2 + b2) * 0.5;
        bool arc = false;
        const double denom = A.dir.x * B.dir.y - A.dir.y * B.dir.x;
        if (std::fabs(denom) > 0.15) {
            const Vec2 d = b2 - a2;
            const double t = (d.x * B.dir.y - d.y * B.dir.x) / denom;
            const Vec2 hit = a2 + A.dir * t;
            // A kerb-return control sits NEAR the corner. A far intersection
            // (near-parallel arms on a generated graph) would sweep the arc
            // across the pad and self-intersect the loop — earcut then spins
            // its recovery passes (the 14-minute city hang). Straight chord.
            if ((hit - (a2 + b2) * 0.5).length() < 0.75 * chord) { c2 = hit; arc = true; }
        }
        const int nArc = arc ? std::max(2, std::min(10, static_cast<int>(chord / 1.5))) : 1;
        for (int k = 1; k < nArc; ++k) {
            const double t = k / static_cast<double>(nArc);
            const double w0 = (1 - t) * (1 - t), w1 = 2 * t * (1 - t), w2 = t * t;
            const Vec2 p = a2 * w0 + c2 * w1 + b2 * w2;
            push(Vec3(p.x, a3.y + (b3.y - a3.y) * t, p.y), -1);
        }
    }
    if (loop.size() >= 2 && (loop.front() - loop.back()).lengthSquared() < 1e-10) {
        loop.pop_back();
        isMouth.pop_back();
        armOf.pop_back();
    }
    const int n = static_cast<int>(loop.size());
    if (n < 3) return mesh;

    // Normalize to CCW in plan view (positive signed area) so the inside
    // tests, the stitch orientation, and earcut all agree.
    {
        double area2 = 0;
        for (int i = 0; i < n; ++i) {
            const Vec3& p = loop[i];
            const Vec3& q = loop[(i + 1) % n];
            area2 += p.x * q.z - q.x * p.z;
        }
        if (area2 < 0) {
            std::reverse(loop.begin(), loop.end());
            std::reverse(isMouth.begin(), isMouth.end());
            std::reverse(armOf.begin(), armOf.end());
        }
    }
    // Footprint for the curb/sidewalk union: the boundary loop with each mouth
    // vertex nudged ~5 cm OUT along its arm, so the pad footprint OVERLAPS the
    // body ribbon instead of sharing an exactly-collinear edge (polygonUnion's
    // documented weak case).
    if (footprintOut) {
        footprintOut->clear();
        footprintOut->reserve(loop.size());
        for (int i = 0; i < n; ++i) {
            Vec3 p = loop[i];
            if (armOf[i] >= 0) {
                const Vec2& d = arms[armOf[i]].dir;
                p.x += d.x * 0.05;
                p.z += d.y * 0.05;
            }
            footprintOut->push_back(p);
        }
    }

    // --- Interior grid, oriented to the widest arm ------------------------
    const double h = 3.0;                      // cell size (~lane scale)
    int widest = 0;
    double widestW = 0;
    for (int i = 0; i < N; ++i) {
        const double w = (arms[i].mouth.front() - arms[i].mouth.back()).length();
        if (w > widestW) { widestW = w; widest = i; }
    }
    const Vec2 ex = normalize(arms[widest].dir);
    const Vec2 ez(-ex.y, ex.x);
    auto toGrid = [&](const Vec2& p) {
        const Vec2 d = p - ctr;
        return Vec2(dot(d, ex), dot(d, ez));
    };
    auto toWorld = [&](double gx, double gz) { return ctr + ex * gx + ez * gz; };

    auto insideLoop = [&](const Vec2& p) {
        bool in = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            const Vec2 a(loop[i].x, loop[i].z), b(loop[j].x, loop[j].z);
            if ((a.y > p.y) != (b.y > p.y) &&
                p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)
                in = !in;
        }
        return in;
    };
    // Distance to the boundary (the keep margin) — and, separately, a SMOOTH
    // height sample: inverse-distance weighting over the loop vertices, so a
    // grid corner equidistant from two arms BLENDS their heights instead of
    // snapping to whichever is nearest. Nearest-height init put a height CLIFF
    // along the frontier wherever "nearest" switched arms — a steep stitch
    // face on every hilly junction.
    auto boundaryDistance = [&](const Vec2& p) {
        double best = 1e30;
        for (int i = 0; i < n; ++i) {
            const Vec3& A3 = loop[i];
            const Vec3& B3 = loop[(i + 1) % n];
            const Vec2 a(A3.x, A3.z), b(B3.x, B3.z);
            const Vec2 ab = b - a;
            const double l2 = ab.lengthSquared();
            double t = l2 > 1e-12 ? dot(p - a, ab) / l2 : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            best = std::min(best, (a + ab * t - p).length());
        }
        return best;
    };
    auto boundaryHeight = [&](const Vec2& p) {
        double wsum = 0, hsum = 0;
        for (int i = 0; i < n; ++i) {
            const double d2 = (Vec2(loop[i].x, loop[i].z) - p).lengthSquared();
            const double w = 1.0 / (d2 + 0.25);
            wsum += w;
            hsum += w * loop[i].y;
        }
        return wsum > 0 ? hsum / wsum : 0.0;
    };

    double gxMin = 1e30, gxMax = -1e30, gzMin = 1e30, gzMax = -1e30;
    for (const Vec3& p : loop) {
        const Vec2 gq = toGrid(Vec2(p.x, p.z));
        gxMin = std::min(gxMin, gq.x); gxMax = std::max(gxMax, gq.x);
        gzMin = std::min(gzMin, gq.y); gzMax = std::max(gzMax, gq.y);
    }
    const int gi0 = static_cast<int>(std::floor(gxMin / h)) - 1;
    const int gi1 = static_cast<int>(std::ceil(gxMax / h)) + 1;
    const int gj0 = static_cast<int>(std::floor(gzMin / h)) - 1;
    const int gj1 = static_cast<int>(std::ceil(gzMax / h)) + 1;
    const int gw = gi1 - gi0 + 1, gh2 = gj1 - gj0 + 1;

    struct Corner { double y = 0; bool kept = false; bool interior = false; };
    std::vector<Corner> corner(static_cast<std::size_t>(gw) * gh2);
    auto cornerAt = [&](int gi, int gj) -> Corner& {
        return corner[static_cast<std::size_t>(gj - gj0) * gw + (gi - gi0)];
    };
    int keptCorners = 0;
    for (int gj = gj0; gj <= gj1; ++gj)
        for (int gi = gi0; gi <= gi1; ++gi) {
            const Vec2 p = toWorld(gi * h, gj * h);
            if (insideLoop(p) && boundaryDistance(p) >= 0.45 * h) {
                Corner& c = cornerAt(gi, gj);
                c.kept = true;
                c.y = boundaryHeight(p);       // smooth blend of arm heights
                ++keptCorners;
            }
        }

    // Kept CELLS (all four corners kept) and the frontier of their union.
    auto cellKept = [&](int gi, int gj) {
        return gi >= gi0 && gi + 1 <= gi1 && gj >= gj0 && gj + 1 <= gj1 &&
               cornerAt(gi, gj).kept && cornerAt(gi + 1, gj).kept &&
               cornerAt(gi + 1, gj + 1).kept && cornerAt(gi, gj + 1).kept;
    };
    int keptCells = 0;
    for (int gj = gj0; gj < gj1; ++gj)
        for (int gi = gi0; gi < gi1; ++gi)
            if (cellKept(gi, gj)) ++keptCells;

    if (keptCells == 0) {                      // pad too small for a grid
        if (std::getenv("RT_PAD_DEBUG")) std::fprintf(stderr, "[pad] fallback: keptCells=0 (corners=%d)\n", keptCorners);
        return junctionPadEarcut(loop, mu, color);
    }

    // Frontier: the union outline of kept cells, walked with the region on the
    // left (CCW). Directed edges between grid corners; shared edges cancel.
    std::map<std::pair<int, int>, std::pair<int, int>> nextOf;
    int frontierEdges = 0;
    for (int gj = gj0; gj < gj1; ++gj)
        for (int gi = gi0; gi < gi1; ++gi) {
            if (!cellKept(gi, gj)) continue;
            auto add = [&](int ax, int az, int bx, int bz) {
                nextOf[{ax, az}] = {bx, bz};
                ++frontierEdges;
            };
            if (!cellKept(gi, gj - 1)) add(gi, gj, gi + 1, gj);          // bottom
            if (!cellKept(gi + 1, gj)) add(gi + 1, gj, gi + 1, gj + 1);  // right
            if (!cellKept(gi, gj + 1)) add(gi + 1, gj + 1, gi, gj + 1);  // top
            if (!cellKept(gi - 1, gj)) add(gi, gj + 1, gi, gj);          // left
        }
    // A corner can carry TWO outgoing frontier edges (diagonal cell contact) —
    // the map keeps one and the stitch below detects the broken cycle and
    // falls back. Same for multiple loops (holes / split regions).
    std::vector<std::pair<int, int>> frontier;
    {
        auto start = nextOf.begin()->first;
        auto cur = start;
        for (std::size_t guard = 0; guard <= nextOf.size(); ++guard) {
            frontier.push_back(cur);
            auto it = nextOf.find(cur);
            if (it == nextOf.end()) break;
            cur = it->second;
            if (cur == start) break;
        }
        if (frontier.size() != nextOf.size() ||
            static_cast<int>(nextOf.size()) != frontierEdges || frontier.size() < 3) {
            if (std::getenv("RT_PAD_DEBUG"))
                std::fprintf(stderr, "[pad] fallback: frontier=%zu nextOf=%zu edges=%d cells=%d\n",
                             frontier.size(), nextOf.size(), frontierEdges, keptCells);
            return junctionPadEarcut(loop, mu, color);
        }
    }

    // Laplace-relax the interior corner heights (frontier corners hold the
    // nearest-boundary values): the pad's surface bends smoothly between the
    // arms instead of creasing — the plan's "interior heights by smooth
    // interpolation from the fixed boundary".
    for (int gj = gj0; gj <= gj1; ++gj)
        for (int gi = gi0; gi <= gi1; ++gi) {
            Corner& c = cornerAt(gi, gj);
            c.interior = c.kept && gi > gi0 && gi < gi1 && gj > gj0 && gj < gj1 &&
                         cornerAt(gi - 1, gj).kept && cornerAt(gi + 1, gj).kept &&
                         cornerAt(gi, gj - 1).kept && cornerAt(gi, gj + 1).kept;
        }
    for (int it2 = 0; it2 < 32; ++it2) {
        for (int gj = gj0 + 1; gj < gj1; ++gj)
            for (int gi = gi0 + 1; gi < gi1; ++gi) {
                Corner& c = cornerAt(gi, gj);
                if (!c.interior) continue;
                c.y = (cornerAt(gi - 1, gj).y + cornerAt(gi + 1, gj).y +
                       cornerAt(gi, gj - 1).y + cornerAt(gi, gj + 1).y) * 0.25;
            }
    }

    // --- Emit: ONE shared-vertex mesh (loop verts + grid corners) ---------
    std::vector<Vertex> verts;
    std::vector<uint32_t> idx;
    auto addVert = [&](const Vec3& p) {
        Vertex v;
        v.position = p;
        v.normal = Vec3(0, 0, 0);              // accumulated from faces below
        v.tangent = Vec3(ex.x, 0, ex.y);
        v.u = mu;
        v.v = 0.0f;
        v.color = color;
        verts.push_back(v);
        return static_cast<uint32_t>(verts.size() - 1);
    };
    std::vector<uint32_t> loopIdx(n);
    for (int i = 0; i < n; ++i) loopIdx[i] = addVert(loop[i]);
    std::map<std::pair<int, int>, uint32_t> gridIdx;
    auto gridVert = [&](int gi, int gj) {
        const auto k = std::make_pair(gi, gj);
        auto it = gridIdx.find(k);
        if (it != gridIdx.end()) return it->second;
        const Vec2 p = toWorld(gi * h, gj * h);
        const uint32_t vi = addVert(Vec3(p.x, cornerAt(gi, gj).y, p.y));
        gridIdx.emplace(k, vi);
        return vi;
    };
    // 2D CCW check in plan view (up-facing triangle before winding fix).
    auto ccw2 = [&](uint32_t a, uint32_t b, uint32_t c) {
        const Vec3& A = verts[a].position;
        const Vec3& B = verts[b].position;
        const Vec3& C = verts[c].position;
        return (B.x - A.x) * (C.z - A.z) - (C.x - A.x) * (B.z - A.z);
    };
    bool stitchOk = true;
    auto tri = [&](uint32_t a, uint32_t b, uint32_t c, bool guard) {
        const double a2 = ccw2(a, b, c);
        if (std::fabs(a2) < 1e-9) return;      // zero-area: skip, never emit
        if (guard && a2 < -1e-6) stitchOk = false;   // stitch folded over
        // Engine winding: outward normal = cross(c - a, b - a), so a plan-CCW
        // triple emitted IN ORDER faces up; a CW triple emits swapped.
        idx.push_back(a);
        idx.push_back(a2 > 0 ? b : c);
        idx.push_back(a2 > 0 ? c : b);
    };
    // Interior quads, split along the shorter 3D diagonal.
    for (int gj = gj0; gj < gj1; ++gj)
        for (int gi = gi0; gi < gi1; ++gi) {
            if (!cellKept(gi, gj)) continue;
            const uint32_t v00 = gridVert(gi, gj), v10 = gridVert(gi + 1, gj);
            const uint32_t v11 = gridVert(gi + 1, gj + 1), v01 = gridVert(gi, gj + 1);
            const double d0 = (verts[v00].position - verts[v11].position).lengthSquared();
            const double d1 = (verts[v10].position - verts[v01].position).lengthSquared();
            if (d0 <= d1) { tri(v00, v10, v11, false); tri(v00, v11, v01, false); }
            else          { tri(v00, v10, v01, false); tri(v10, v11, v01, false); }
        }
    // Annulus stitch: outer loop (CCW) to frontier (CCW), advancing whichever
    // cursor makes the shorter diagonal. Every loop vertex is consumed by
    // construction — the EXACT-K guarantee (no pruned mouth points, ever).
    {
        const int m = static_cast<int>(frontier.size());
        std::vector<uint32_t> inIdx(m);
        for (int j = 0; j < m; ++j) inIdx[j] = gridVert(frontier[j].first, frontier[j].second);
        int i0 = 0, j0 = 0;
        double best = 1e30;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j) {
                const double d = (verts[loopIdx[i]].position - verts[inIdx[j]].position).lengthSquared();
                if (d < best) { best = d; i0 = i; j0 = j; }
            }
        int i = 0, j = 0;
        auto O = [&](int k) { return loopIdx[(i0 + k) % n]; };
        auto F = [&](int k) { return inIdx[(j0 + k) % m]; };
        // Advance by NORMALIZED ARC LENGTH, not nearest diagonal: on an
        // elongated pad (a compound junction pair) greedy-nearest lets one
        // cursor lag, then its catch-up triangles sweep ACROSS the interior —
        // positive-area overlap the fold guard cannot see (measured: the whole
        // link double-covered). Arc-synchronized cursors walk the two loops in
        // lockstep around the ring, so the stitch stays inside the annulus.
        std::vector<double> arcO(n + 1, 0.0), arcF(m + 1, 0.0);
        for (int k = 0; k < n; ++k)
            arcO[k + 1] = arcO[k] + (verts[O(k + 1)].position - verts[O(k)].position).length();
        for (int k = 0; k < m; ++k)
            arcF[k + 1] = arcF[k] + (verts[F(k + 1)].position - verts[F(k)].position).length();
        const double totO = std::max(1e-9, arcO[n]), totF = std::max(1e-9, arcF[m]);
        while (i < n || j < m) {
            const bool canO = i < n, canF = j < m;
            const double pO = canO ? arcO[i + 1] / totO : 2.0;
            const double pF = canF ? arcF[j + 1] / totF : 2.0;
            if (canO && (!canF || pO <= pF)) { tri(O(i), O(i + 1), F(j), true); ++i; }
            else                             { tri(O(i), F(j + 1), F(j), true); ++j; }
        }
    }
    if (!stitchOk) {                           // stitch folded: odd geometry
        if (std::getenv("RT_PAD_DEBUG")) std::fprintf(stderr, "[pad] fallback: stitch folded\n");
        return junctionPadEarcut(loop, mu, color);
    }

    // Per-vertex normals accumulated from faces; drivable pad faces up.
    for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
        const Vec3& A = verts[idx[t]].position;
        const Vec3& B = verts[idx[t + 1]].position;
        const Vec3& C = verts[idx[t + 2]].position;
        Vec3 fn = cross(C - A, B - A);
        if (fn.y < 0) fn = fn * -1.0;
        verts[idx[t]].normal = verts[idx[t]].normal + fn;
        verts[idx[t + 1]].normal = verts[idx[t + 1]].normal + fn;
        verts[idx[t + 2]].normal = verts[idx[t + 2]].normal + fn;
    }
    for (Vertex& v : verts)
        v.normal = v.normal.lengthSquared() > 1e-12 ? normalize(v.normal) : Vec3(0, 1, 0);

    mesh.vertices = std::move(verts);
    mesh.indices = std::move(idx);
    return mesh;
}

// ---------------------------------------------------------------------------
// Roads-v2 S5: the raised curb + sidewalk BAND (ownership: asphalt = bodies +
// pads; this band owns everything from the asphalt edge to the sidewalk's
// outer face). Swept along each closed union-outline loop, riding the loop's
// RIGHT normal (outward from the asphalt for CCW outers AND CW holes), with
// the corner logic proven in the old weld: gentle corners miter, sharp OPENING
// corners bevel with a fan, PINCHING corners meet at the true intersection.
RenderMesh sweepCurbSidewalkBand(const std::vector<Poly2>& loops,
                                 const std::function<double(double, double)>& edgeHeight,
                                 double sidewalkWidth, double curbHeight,
                                 const std::vector<std::pair<Vec2, Vec2>>* mouthGaps) {
    RenderMesh mesh;
    if (sidewalkWidth <= 0.0) return mesh;
    const Vec3 walkColor(0.62, 0.62, 0.60);
    const Vec3 curbColor(0.48, 0.48, 0.47);
    const double drop = 0.35;             // outer face tucks under the ground
    auto rnorm = [](const Vec2& e) {
        Vec2 n(e.y, -e.x);
        const double l = n.length();
        return l < 1e-9 ? Vec2(0, 0) : n * (1.0 / l);
    };
    auto P3 = [&](const Vec2& v, double dh) {
        return Vec3(v.x, edgeHeight(v.x, v.y) + dh, v.y);
    };
    for (const Poly2& loop : loops) {
        const int m = static_cast<int>(loop.size());
        if (m < 3) continue;
        struct Corner { Vec2 prevPt, nextPt; bool bevel; };
        std::vector<Corner> oc(m);
        for (int i = 0; i < m; ++i) {
            const Vec2 e0 = loop[i] - loop[(i + m - 1) % m];
            const Vec2 e1 = loop[(i + 1) % m] - loop[i];
            const Vec2 n0 = rnorm(e0), n1 = rnorm(e1);
            const Vec2 bis = n0 + n1;
            const double bl = bis.length();
            const Vec2 mm = bl < 1e-9 ? n1 : bis * (1.0 / bl);
            const double cosH = dot(mm, n1);
            // A LEFT turn OPENS the band (fan/bevel closes the gap); a RIGHT
            // turn PINCHES it (edges meet at the offset lines' intersection).
            const bool opens = (e0.x * e1.y - e0.y * e1.x) > 0.0;
            if (!opens) {
                const Vec2 q = loop[i] + mm * (sidewalkWidth / std::max(0.2, cosH));
                oc[i] = { q, q, false };
            } else if (cosH >= 0.5) {
                const Vec2 q = loop[i] + mm * (sidewalkWidth / cosH);
                oc[i] = { q, q, false };
            } else {
                oc[i] = { loop[i] + n0 * sidewalkWidth,
                          loop[i] + n1 * sidewalkWidth, true };
            }
        }
        // Arc length -> negative-u slab UV: the shader scores sidewalk joints
        // PERPENDICULAR to the curb, following its curvature, and negative u
        // keeps the band inside the existing "mu < 0.98 = sidewalk" gate.
        std::vector<double> sArc(m + 1, 0.0);
        for (int i = 0; i < m; ++i)
            sArc[i + 1] = sArc[i] + (loop[(i + 1) % m] - loop[i]).length();
        auto inMouthGap = [&](const Vec2& p) {
            if (!mouthGaps) return false;
            for (const auto& gseg : *mouthGaps) {
                const Vec2 ab = gseg.second - gseg.first;
                const double l2 = ab.lengthSquared();
                double t = l2 > 1e-12 ? dot(p - gseg.first, ab) / l2 : 0.0;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                if ((gseg.first + ab * t - p).length() < 0.8) return true;
            }
            return false;
        };
        for (int i = 0; i < m; ++i) {
            const int j = (i + 1) % m;
            const Vec2& a = loop[i];
            const Vec2& b = loop[j];
            if ((b - a).length() < 1e-9) continue;
            if (inMouthGap((a + b) * 0.5)) continue;   // never curb a ramp shut
            const Vec2 ao = oc[i].nextPt, bo = oc[j].prevPt;
            const Vec2 rn = rnorm(b - a);
            const Vec3 eo3(rn.x, 0, rn.y);   // outward, per edge
            const Vec3 aT = P3(a, curbHeight), bT = P3(b, curbHeight);
            const Vec3 aR = P3(a, 0), bR = P3(b, 0);
            const Vec3 aoT = P3(ao, curbHeight), boT = P3(bo, curbHeight);
            const Vec3 aoB = P3(ao, -drop), boB = P3(bo, -drop);
            MeshBuilder::emitQuad(mesh, aR, bR, bT, aT, eo3 * -1.0, curbColor);  // curb lip
            const float ua = static_cast<float>(-sArc[i]);
            const float ub = static_cast<float>(-sArc[i + 1]);
            MeshBuilder::emitTriUV(mesh, aT, bT, boT, Vec3(0, 1, 0), walkColor,
                                   ua, 0.0f, ub, 0.0f, ub, 1.0f);                // slab top
            MeshBuilder::emitTriUV(mesh, aT, boT, aoT, Vec3(0, 1, 0), walkColor,
                                   ua, 0.0f, ub, 1.0f, ua, 1.0f);
            MeshBuilder::emitQuad(mesh, aoT, boT, boB, aoB, eo3, curbColor);     // outer face
            if (oc[j].bevel) {
                // Fan closing the band around an opening corner + its outer face.
                const Vec3 jT = P3(b, curbHeight);
                const Vec3 p0T = P3(oc[j].prevPt, curbHeight);
                const Vec3 p1T = P3(oc[j].nextPt, curbHeight);
                MeshBuilder::emitTriUV(mesh, jT, p0T, p1T, Vec3(0, 1, 0), walkColor,
                                       ub, 0.0f, ub, 1.0f, ub, 1.0f);
                const Vec2 bev = oc[j].nextPt - oc[j].prevPt;
                if (bev.length() > 1e-9) {
                    const Vec2 bn = rnorm(bev);
                    const Vec3 p0B = P3(oc[j].prevPt, -drop);
                    const Vec3 p1B = P3(oc[j].nextPt, -drop);
                    MeshBuilder::emitQuad(mesh, p0T, p1T, p1B, p0B,
                                          Vec3(bn.x, 0, bn.y), curbColor);
                }
            }
        }
    }
    return mesh;
}

}  // namespace engine
