// The swept freeway deck (docs/road-mesher-research.md §2), tested in isolation
// before it is wired into the loader. Proves the sweep produces what the earcut
// union mesher could not: a shared-vertex lattice (not a soup), a surface a car
// can drive without steps, real per-road UV so the shader paints asphalt not
// sidewalk concrete, and correct banking + flare from the spine's own channels.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/road_lattice.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include <cmath>
#include <map>
#include <vector>

using namespace engine;

namespace {

// A straight elevated freeway deck: +9 m, 14 m half-width, `cs` cross-slope.
UnionSpine deckSpine(double cs, double hw0, double hw1) {
    UnionSpine s;
    s.klass = RoadClass::Freeway;
    const int n = 24;
    for (int i = 0; i <= n; ++i) {
        const double t = i / static_cast<double>(n);
        s.points.push_back(Vec2(t * 240.0, 0.0));
        s.yAbs.push_back(9.0);
        s.hw.push_back(hw0 + (hw1 - hw0) * t);
        s.crossSlope.push_back(cs);
    }
    return s;
}

double triAspect(const Vec3& a, const Vec3& b, const Vec3& c) {
    const double longest = std::max({ (b - a).length(), (c - b).length(), (a - c).length() });
    const double area = cross(b - a, c - a).length() * 0.5;
    if (area < 1e-12) return 1e9;
    return longest / std::max(1e-9, 2.0 * area / longest);
}

}  // namespace

// A shared lattice, not a soup: V/T well under 1, no vertex the apex of a fan,
// no slivers, no degenerate faces — the exact defects measured on weldSolid
// (V/T 2.9, a 457-fan, 68.8% slivers).
TEST_CASE(freeway_deck_lattice_is_clean) {
    RenderMesh m = sweepRoadLattice(deckSpine(0.0, 14, 14), freewayDeckProfile(3), nullptr);
    const std::size_t tris = m.indices.size() / 3;
    CHECK(tris > 0);
    CHECK(static_cast<double>(m.vertices.size()) / tris < 0.7);

    std::map<uint32_t, int> valence;
    for (uint32_t idx : m.indices) ++valence[idx];
    int worstFan = 0;
    for (const auto& [k, n] : valence) worstFan = std::max(worstFan, n);
    CHECK(worstFan <= 6);

    int slivers = 0, degenerate = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        if (cross(b - a, c - a).length() < 1e-9) ++degenerate;
        else if (triAspect(a, b, c) > 20.0) ++slivers;
    }
    CHECK(degenerate == 0);
    CHECK(100.0 * slivers / static_cast<double>(tris) < 5.0);
}

// A car drives the centreline without falling through a hole or hitting a step —
// the surface is continuous by construction (rings placed along arc length),
// not reconstructed per-vertex.
TEST_CASE(freeway_deck_is_drivable) {
    RenderMesh m = sweepRoadLattice(deckSpine(0.0, 14, 14), freewayDeckProfile(3), nullptr);
    std::vector<Vec3> path;
    for (int i = 0; i <= 24; ++i) path.push_back(Vec3(i * 10.0, 9.0, 0.0));
    driveprobe::Report rep;
    driveprobe::drivePath(m, path, rep);
    CHECK(rep.samples > 50);
    CHECK(rep.holes == 0);
    CHECK(rep.steps == 0);
    CHECK(rep.blocked == 0);      // no wall on the deck top
}

// The deck carries road-local UV: the carriageway columns land in mu [1,3] with
// the centreline at 2 and the edges at 1 and 3 — so surfRoadMarkings paints
// asphalt + lane lines, not the mu=0 sidewalk-concrete the earcut sheet gave it.
TEST_CASE(freeway_deck_has_carriageway_uv) {
    RenderMesh m = sweepRoadLattice(deckSpine(0.0, 14, 14), freewayDeckProfile(3), nullptr);
    float muMin = 9, muMax = -9;
    bool sawCentre = false;
    for (const Vertex& v : m.vertices) {
        muMin = std::min(muMin, v.u);
        muMax = std::max(muMax, v.u);
        if (std::fabs(v.u - 2.0f) < 1e-3f) sawCentre = true;
        CHECK(v.u >= 0.98f);      // never the sidewalk-concrete branch
    }
    CHECK(std::fabs(muMin - 1.0f) < 1e-3f);   // right edge line
    CHECK(std::fabs(muMax - 3.0f) < 1e-3f);   // left edge line
    CHECK(sawCentre);                          // centreline column present
}

// Superelevation banks the deck: with fwd = +X and +left = +Z, a positive
// cross-slope raises the +Z edge above the -Z edge. The tilt comes from the
// spine's own crossSlope channel, applied as the deck plane is placed.
TEST_CASE(freeway_deck_banks_with_cross_slope) {
    RenderMesh m = sweepRoadLattice(deckSpine(0.06, 14, 14), freewayDeckProfile(3), nullptr);
    double yPos = -1e9, yNeg = -1e9;
    for (const Vertex& v : m.vertices) {
        if (v.position.z > 12.0) yPos = std::max(yPos, static_cast<double>(v.position.y));
        if (v.position.z < -12.0) yNeg = std::max(yNeg, static_cast<double>(v.position.y));
    }
    CHECK(yPos > yNeg + 0.5);     // 14 m * 0.06 ~ 0.84 m each way
}

// The deck flares with the spine's per-point half-width (an aux-lane / gore
// widening): the ribbon is wider at the flared end than at the start.
TEST_CASE(freeway_deck_flares_with_half_width) {
    RenderMesh m = sweepRoadLattice(deckSpine(0.0, 8, 16), freewayDeckProfile(3), nullptr);
    double wStart = 0, wEnd = 0;
    for (const Vertex& v : m.vertices) {
        if (v.position.x < 5.0) wStart = std::max(wStart, std::fabs((double)v.position.z));
        if (v.position.x > 235.0) wEnd = std::max(wEnd, std::fabs((double)v.position.z));
    }
    CHECK(wStart > 7.0 && wStart < 9.0);      // ~8 m half-width at the start
    CHECK(wEnd > 15.0 && wEnd < 17.0);        // ~16 m at the flared end
}

// The whole viaduct cross-section: the deck reads as a STRUCTURE, not black
// pavement — it has a down-facing soffit under it and vertical fascia/parapet
// faces, none of them degenerate. A lane is drivable (parapets at the verge and
// the median at the centre don't block an offset lane), and banking/flare still
// hold with the furniture attached.
TEST_CASE(freeway_full_section_has_structure_and_drives) {
    RenderMesh m = sweepFreewayDeck(deckSpine(0.0, 14, 14), nullptr);
    const std::size_t tris = m.indices.size() / 3;
    CHECK(tris > 0);

    int up = 0, down = 0, lateral = 0, degenerate = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        Vec3 gn = cross(b - a, c - a);
        if (gn.length() < 1e-9) { ++degenerate; continue; }
        gn = normalize(gn);
        if (gn.y > 0.5) ++up;
        else if (gn.y < -0.5) ++down;     // soffit underside
        else ++lateral;                    // fascia / parapet faces
    }
    CHECK(degenerate == 0);
    CHECK(up > 0);
    CHECK(down > 0);          // it has an underside
    CHECK(lateral > 0);       // it has walls

    // Drive a LANE (offset from centre), the way the nav's one-way carriageway
    // chains do: clear of the median at centre and the parapets at the verge.
    std::vector<Vec3> lane;
    for (int i = 0; i <= 24; ++i) lane.push_back(Vec3(i * 10.0, 9.0, 6.0));  // +6 m off centre
    driveprobe::Report rep;
    driveprobe::drivePath(m, lane, rep);
    CHECK(rep.samples > 50);
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);      // no wall in the lane
}

// The blocked-merge fix, as data: an edge parapet GAPS over a gore window, so a
// ramp can merge where otherwise a 0.85 m wall stood across its path. With the
// gap, no parapet geometry exists over that arc-length; without it, the wall is
// continuous.
TEST_CASE(freeway_parapet_gaps_over_a_gore) {
    // Parapets ride the +Z / -Z verges (hw = 14). Count wall-height vertices
    // (y ~ deck+0.85) in an x-window, with and without a gap there.
    auto wallVertsInWindow = [](const RenderMesh& m, double x0, double x1) {
        int n = 0;
        for (const Vertex& v : m.vertices)
            if (v.position.x > x0 && v.position.x < x1 && v.position.y > 9.5 &&
                std::fabs(v.position.z) > 12.0)
                ++n;
        return n;
    };
    const UnionSpine s = deckSpine(0.0, 14, 14);   // 0..240 m along X

    RenderMesh solid = sweepFreewayDeck(s, nullptr);
    CHECK(wallVertsInWindow(solid, 100, 140) > 0);       // continuous wall

    std::vector<GapWindow> gaps = { { 100.0, 140.0 } };  // gore window (arc length)
    RenderMesh gapped = sweepFreewayDeck(s, nullptr, 3.0, 0.5, &gaps);
    CHECK(wallVertsInWindow(gapped, 105, 135) == 0);     // parapet opened for the merge
    CHECK(wallVertsInWindow(gapped, 0, 40) > 0);         // still there elsewhere
}
