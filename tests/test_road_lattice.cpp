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
    CHECK(rep.steps == 0);        // the deck sweep itself is continuous
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

// The junction patch (road-mesher-research.md §3). Four arms arrive at DIFFERENT
// heights — the exact case where weldSolid's nearest-spine field STEPS across the
// medial axis to a 164% grade. The Coons patch INTERPOLATES all four, so the pad
// is a gentle ramp with no step, and it is a clean shared-vertex all-quad lattice.
TEST_CASE(junction_coons_patch_is_smooth_and_all_quad) {
    auto side = [](Vec3 a, Vec3 b, int n) {
        std::vector<Vec3> s;
        for (int i = 0; i <= n; ++i) s.push_back(a + (b - a) * (i / static_cast<double>(n)));
        return s;
    };
    // A 20 m square pad, corners at 0 / 0.5 / 0.5 / 1.0 m — ~1 m of relief across
    // 20 m, i.e. a ~5% surface. Anything steep in the pad is a mesher step.
    const Vec3 SW(-10, 0.0, -10), SE(10, 0.5, -10), NW(-10, 0.5, 10), NE(10, 1.0, 10);
    std::vector<Vec3> bottom = side(SW, SE, 8), top = side(NW, NE, 8);
    std::vector<Vec3> left = side(SW, NW, 8), right = side(SE, NE, 8);
    RenderMesh m = coonsPatch(bottom, right, top, left);

    const std::size_t tris = m.indices.size() / 3;
    CHECK(tris > 0);
    CHECK(static_cast<double>(m.vertices.size()) / tris < 0.7);   // shared lattice

    int steep = 0, degenerate = 0;
    double worst = 0.0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        const Vec3 nrm = cross(b - a, c - a);
        const double ny = std::fabs(nrm.y);
        if (ny < 1e-9) { ++degenerate; continue; }
        const double g = std::sqrt(nrm.x * nrm.x + nrm.z * nrm.z) / ny;
        worst = std::max(worst, g);
        if (g > 0.25) ++steep;
    }
    CHECK(degenerate == 0);
    CHECK(steep == 0);            // no medial-axis step — the 164% face is gone
    CHECK(worst < 0.15);         // the whole pad is as gentle as the arms imply

    // The pad interpolates: its centre sits at the mean of the corner heights.
    double cy = 0;
    for (const Vertex& v : m.vertices)
        if (std::fabs(v.position.x) < 1e-6 && std::fabs(v.position.z) < 1e-6) cy = v.position.y;
    CHECK(std::fabs(cy - 0.5) < 1e-6);
}

namespace {
UnionSpine straightStreet(double hw) {
    UnionSpine s;
    s.klass = RoadClass::Local;
    for (int i = 0; i <= 24; ++i) s.points.push_back(Vec2(i * 8.0, 0.0));
    s.halfWidth = hw;
    return s;                                     // no yAbs -> drapes on the ground
}
}  // namespace

// Stage 3a: the swept STREET body — a clean shared-vertex lattice with a drivable
// carriageway and a raised sidewalk + curb, not the 2-triangle earcut ribbon.
TEST_CASE(street_body_is_clean_drivable_with_sidewalk) {
    auto flat = [](double, double) { return 0.0; };
    RenderMesh m = sweepRoadLattice(straightStreet(3.6), streetProfile(1, 3.0, 0.15), flat, 2.0);

    const std::size_t tris = m.indices.size() / 3;
    CHECK(tris > 0);
    CHECK(static_cast<double>(m.vertices.size()) / tris < 0.7);
    std::map<uint32_t, int> valence;
    for (uint32_t idx : m.indices) ++valence[idx];
    int worstFan = 0, slivers = 0, degenerate = 0;
    for (const auto& [k, n] : valence) worstFan = std::max(worstFan, n);
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        if (cross(b - a, c - a).length() < 1e-9) ++degenerate;
        else if (triAspect(a, b, c) > 20.0) ++slivers;
    }
    CHECK(worstFan <= 6);
    CHECK(degenerate == 0);
    CHECK(100.0 * slivers / static_cast<double>(tris) < 5.0);

    // A lane drives clean (offset half the carriageway off centre).
    std::vector<Vec3> lane;
    for (int i = 0; i <= 24; ++i) lane.push_back(Vec3(i * 8.0, 0.0, 1.8));
    driveprobe::Report rep;
    driveprobe::drivePath(m, lane, rep);
    CHECK(rep.samples > 40);
    CHECK(rep.holes == 0);
    CHECK(rep.steps == 0);
    CHECK(rep.blocked == 0);      // sidewalk/curb are at the verge, not the lane

    // The raised sidewalk exists: concrete (mu<1) vertices lifted above the road.
    bool sawSidewalk = false;
    for (const Vertex& v : m.vertices)
        if (v.position.y > 0.1 && std::fabs(v.position.z) > 3.7 && v.u < 1.0) sawSidewalk = true;
    CHECK(sawSidewalk);
}

// The whole point of the rewrite (Glenn: "no geo to conform"): on rolling terrain
// the street surface now UNDULATES with the ground, because it has interior
// vertices to drape — and stays gentle where the ground is gentle.
TEST_CASE(street_body_conforms_to_hills) {
    auto hills = [](double x, double z) {
        return 1.5 * std::sin(x * 0.02) + 1.2 * std::cos(z * 0.017);
    };
    RenderMesh m = sweepRoadLattice(straightStreet(3.6), streetProfile(1, 3.0, 0.15), hills, 2.0);

    double ymin = 1e9, ymax = -1e9;
    int steep = 0;
    for (const Vertex& v : m.vertices) {
        ymin = std::min(ymin, static_cast<double>(v.position.y));
        ymax = std::max(ymax, static_cast<double>(v.position.y));
    }
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        const Vec3 nrm = cross(b - a, c - a);
        if (nrm.y < 1e-6) continue;                        // curb faces / walls
        const double g = std::sqrt(nrm.x * nrm.x + nrm.z * nrm.z) / std::fabs(nrm.y);
        if (g > 0.25) ++steep;
    }
    CHECK(ymax - ymin > 0.5);     // the surface follows the ~1.5 m of relief
    CHECK(steep == 0);            // and no bump the terrain didn't put there
}

// Stage 3b: node -> Coons patch. Four arms meet at DIFFERENT heights (the exact
// hilly-cross case that steps to 164% under nearest-spine). The adapter orders
// them, snaps the kerb corners, and hands the four mouths to coonsPatch, so the
// pad is a smooth all-quad grid that matches every arm.
TEST_CASE(junction_patch_4way_is_smooth_all_quad) {
    auto arm = [](Vec2 d, double R, double hw, double h, int K) {
        JunctionArm a;
        a.dir = normalize(d);
        const Vec2 c = a.dir * R;               // mouth centre on the junction boundary
        const Vec2 pp(-a.dir.y, a.dir.x);       // +perp = left verge
        for (int i = 0; i <= K; ++i) {
            const double t = i / static_cast<double>(K);   // left -> right
            const Vec2 p = c + pp * (hw * (1 - 2 * t));
            a.mouth.push_back(Vec3(p.x, h, p.y));
        }
        return a;
    };
    const double hw = 10.0;
    std::vector<JunctionArm> arms = {
        arm(Vec2(0, 1), hw, hw, 1.0, 6),    // N
        arm(Vec2(1, 0), hw, hw, 0.5, 6),    // E
        arm(Vec2(0, -1), hw, hw, 0.0, 6),   // S
        arm(Vec2(-1, 0), hw, hw, 0.5, 6),   // W
    };
    RenderMesh m = junctionPatch(arms);
    const std::size_t tris = m.indices.size() / 3;
    CHECK(tris > 0);
    CHECK(static_cast<double>(m.vertices.size()) / tris < 0.7);   // all-quad grid

    int steep = 0, degenerate = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        const Vec3 nrm = cross(b - a, c - a);
        const double ny = std::fabs(nrm.y);
        if (ny < 1e-9) { ++degenerate; continue; }
        if (std::sqrt(nrm.x * nrm.x + nrm.z * nrm.z) / ny > 0.25) ++steep;
    }
    CHECK(degenerate == 0);
    CHECK(steep == 0);            // matches the arms' gentle slope, no medial-axis step

    // The pad reaches each arm: a vertex sits at each arm's mouth centre height.
    auto sawHeightNear = [&](double x, double z, double y) {
        for (const Vertex& v : m.vertices)
            if (std::fabs(v.position.x - x) < 0.6 && std::fabs(v.position.z - z) < 0.6 &&
                std::fabs(v.position.y - y) < 0.1) return true;
        return false;
    };
    CHECK(sawHeightNear(0, hw, 1.0));      // N mouth centre at y=1
    CHECK(sawHeightNear(0, -hw, 0.0));     // S mouth centre at y=0
}
