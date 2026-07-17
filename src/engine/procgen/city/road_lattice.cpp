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
                            const std::vector<std::vector<double>>* barrierScale) {
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
            double scale = 1.0;
            if (col.barrier >= 0 && barrierScale &&
                i < static_cast<int>(barrierScale->size()) &&
                col.barrier < static_cast<int>((*barrierScale)[i].size()))
                scale = (*barrierScale)[i][col.barrier];
            const double deckY = rg.yBase + lateral * rg.cs;    // banked deck plane
            Vertex& v = verts[i * P + j];
            v.position = c3 + left3 * lateral;
            v.position.y = deckY + col.height * scale;
            // Cross-section normal (cnLat along +left, cnVert along up), banked.
            Vec3 nrm = left3 * col.cnLat + bankedUp * col.cnVert;
            v.normal = nrm.lengthSquared() > 1e-12 ? normalize(nrm) : Vec3(0, 1, 0);
            v.tangent = Vec3(rg.fwd.x, 0, rg.fwd.y);
            v.u = col.mu;
            v.v = static_cast<float>(rg.s);
            v.color = col.color;
        }
    }
    MeshBuilder::emitLattice(mesh, { R, P, verts.data() });
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
        col.absOffset = 0.0;
        col.height = 0.0;
        col.cnLat = 0.0;
        col.cnVert = 1.0;                                    // drivable, faces up
        // Carriageway paint coord: lateral -hw..+hw -> mu 1..3, centre = 2.
        col.mu = static_cast<float>(2.0 + frac);
        col.color = asphalt;
        prof.cols.push_back(col);
    }
    return prof;
}

}  // namespace engine
