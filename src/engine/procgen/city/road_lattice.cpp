#include "road_lattice.h"

#include "../../mesh_builder.h"      // MeshBuilder::emitLattice
#include <algorithm>
#include <cmath>

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

    const int R = std::max(1, static_cast<int>(std::ceil(total / std::max(0.25, ringStep))));
    for (int k = 0; k <= R; ++k) {
        const double s = total * k / R;
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
    // Central-difference tangents; left = tangent rotated +90 in XZ.
    for (std::size_t i = 0; i < rings.size(); ++i) {
        const Vec2 prev = rings[i == 0 ? 0 : i - 1].c;
        const Vec2 next = rings[i + 1 < rings.size() ? i + 1 : i].c;
        Vec2 d = next - prev;
        if (d.lengthSquared() < 1e-12) d = Vec2(1, 0);
        rings[i].fwd = normalize(d);
        rings[i].left = Vec2(-rings[i].fwd.y, rings[i].fwd.x);
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
            const double lateral = col.edgeFrac * rg.hw + col.absOffset;
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

    // Piers under the ELEVATED spans of the mainline: a column every ~24 m of
    // raised run, from the ground up to the deck soffit. deckY here is the
    // authored spine height; the pier stops at deckY - thickness.
    if (deckSpine.yAbs.size() == deckSpine.points.size()) {
        std::vector<Vec2> cl = deckSpine.points;
        std::vector<double> dy = deckSpine.yAbs;
        std::vector<int> at;
        double since = 1e9;
        for (std::size_t i = 0; i < cl.size(); ++i) {
            const double g = p.ground ? p.ground(cl[i].x, cl[i].y) : 0.0;
            const double clr = dy[i] - g;
            const double seg = i == 0 ? 0.0 : (cl[i] - cl[i - 1]).length();
            since += seg;
            if (clr > 2.0 && since >= 24.0) { at.push_back(static_cast<int>(i)); since = 0.0; }
        }
        if (!at.empty()) {
            MeshBuilder::append(mesh, bridgePiers(cl, dy, at, 2.0, 1.4, p.deckThickness,
                                                  Vec3(0.45, 0.46, 0.48), p.ground));
            if (p.pierBasesOut)
                for (int i : at) p.pierBasesOut->push_back(cl[i]);
        }
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

namespace {
Vec3 ringCentre(const std::vector<Vec3>& r) {
    Vec3 c(0, 0, 0);
    for (const Vec3& p : r) c = c + p;
    return r.empty() ? c : c * (1.0 / r.size());
}
}  // namespace

RenderMesh junctionPatch(std::vector<JunctionArm> arms, float mu, const Vec3& color) {
    RenderMesh mesh;
    const int N = static_cast<int>(arms.size());
    if (N <= 2) return mesh;                          // body runs straight through

    // Order arms CCW by bearing so adjacent arms share a kerb corner.
    std::sort(arms.begin(), arms.end(), [](const JunctionArm& a, const JunctionArm& b) {
        return std::atan2(a.dir.y, a.dir.x) < std::atan2(b.dir.y, b.dir.x);
    });
    // Kerb corner between arm i and its CCW neighbour i+1: arm i's LEFT verge
    // (mouth.front()) meets arm i+1's RIGHT verge (mouth.back()).
    std::vector<Vec3> corner(N);
    for (int i = 0; i < N; ++i) {
        const Vec3& a = arms[i].mouth.front();
        const Vec3& b = arms[(i + 1) % N].mouth.back();
        corner[i] = (a + b) * 0.5;
    }
    // Snap each arm's verge endpoints to the shared kerb corners (arm i's left ->
    // corner[i], right -> corner[i-1]) so adjacent sides meet exactly.
    for (int i = 0; i < N; ++i) {
        if (arms[i].mouth.size() < 2) return mesh;
        arms[i].mouth.front() = corner[i];
        arms[i].mouth.back() = corner[(i - 1 + N) % N];
    }

    // N == 4, opposite arms equal length: a Coons grid, no extraordinary vertex.
    if (N == 4 && arms[0].mouth.size() == arms[2].mouth.size() &&
        arms[1].mouth.size() == arms[3].mouth.size()) {
        auto rev = [](std::vector<Vec3> v) { std::reverse(v.begin(), v.end()); return v; };
        // Corners c0=a0/a1, c1=a1/a2, c2=a2/a3, c3=a3/a0 -> NE,NW,SW,SE.
        // bottom=SW->SE=rev(a3), right=SE->NE=rev(a0), top=NW->NE=a1, left=SW->NW=a2.
        return coonsPatch(rev(arms[3].mouth), rev(arms[0].mouth), arms[1].mouth,
                          arms[2].mouth, mu, color);
    }

    // Stopgap for T (N=3) and N>=5: a centroid fan with INTERPOLATED height, so
    // the pad still doesn't step even though it isn't all-quad yet (the mapped
    // templates are the follow-up). Boundary = arm mouths + kerb corners, CCW.
    std::vector<Vec3> loop;
    for (int i = 0; i < N; ++i) {
        for (const Vec3& p : arms[i].mouth) loop.push_back(p);
        loop.push_back(corner[i]);
    }
    Vec3 c(0, 0, 0);
    for (int i = 0; i < N; ++i) c = c + ringCentre(arms[i].mouth);
    c = c * (1.0 / N);                                // centre at the mean arm centre
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const Vec3& p0 = loop[i];
        const Vec3& p1 = loop[(i + 1) % loop.size()];
        MeshBuilder::emitTri(mesh, c, p0, p1, Vec3(0, 1, 0), color);
    }
    return mesh;
}

}  // namespace engine
