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
                            const std::vector<GapWindow>* gaps) {
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

}  // namespace engine
