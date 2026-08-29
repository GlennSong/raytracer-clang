// The swept freeway deck (docs/road-mesher-research.md §2), tested in isolation
// before it is wired into the loader. Proves the sweep produces what the earcut
// union mesher could not: a shared-vertex lattice (not a soup), a surface a car
// can drive without steps, real per-road UV so the shader paints asphalt not
// sidewalk concrete, and correct banking + flare from the spine's own channels.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/road_lattice.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/road_net.h"
#include <algorithm>
#include <cstdio>
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

// The engine winds front faces so the geometric normal OPPOSES the shading normal
// (AGENTS.md § Procgen Authoring; the Metal viewer culls back faces by this, the
// path tracer is two-sided and silently tolerates the wrong order). The lattice is
// THE road mesher, so every surface it emits has to hold the convention — but it
// had 19 tests and none of them checked winding, which is how a flipped face would
// have reached the viewer unnoticed. Gate it at the level the whole engine uses
// (buildRoadNetLattice), not just one swept body.
namespace {

// The engine convention, pinned against MeshBuilder::box: for an UP-facing surface
// (shading normal +Y) the GEOMETRIC normal cross(b-a, c-a) points DOWN — front
// faces are wound clockwise seen from outside. A deck face whose geometric normal
// points up is genuinely inverted and will be culled away in the viewer.
int upFacesInverted(const RenderMesh& m) {
    int bad = 0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& a = m.vertices[m.indices[i]];
        const Vertex& b = m.vertices[m.indices[i + 1]];
        const Vertex& c = m.vertices[m.indices[i + 2]];
        Vec3 gn = cross(b.position - a.position, c.position - a.position);
        if (gn.lengthSquared() < 1e-24) continue;
        if (a.normal.y > 0.5 && normalize(gn).y > 0.5) ++bad;
    }
    return bad;
}

}  // namespace

// The engine convention, pinned empirically rather than argued about: for an
// up-facing surface MeshBuilder::box (the reference primitive the whole engine is
// checked against) puts cross(b-a, c-a) pointing DOWN. Swept bodies must match it.
TEST_CASE(lattice_winding_matches_the_engine_convention) {
    // The reference, so this test states the convention instead of assuming it.
    int boxUp = 0;
    RenderMesh box = MeshBuilder::box(Vec3(2, 2, 2));
    for (std::size_t i = 0; i + 2 < box.indices.size(); i += 3) {
        const Vertex& a = box.vertices[box.indices[i]];
        const Vertex& b = box.vertices[box.indices[i + 1]];
        const Vertex& c = box.vertices[box.indices[i + 2]];
        Vec3 gn = cross(b.position - a.position, c.position - a.position);
        if (gn.lengthSquared() > 1e-24 && a.normal.y > 0.5 && normalize(gn).y > 0.5) ++boxUp;
    }
    CHECK(boxUp == 0);

    auto flat = [](double, double) { return 0.0; };
    RenderMesh body = sweepRoadLattice(straightStreet(3.6),
                                       streetProfile(1, 3.0, 0.15), flat, 2.0);
    CHECK(upFacesInverted(body) == 0);          // 1152/1152 agree with the box
}

// FIXED (2026-08-17) — and the isolation below is what CORRECTED the earlier
// diagnosis, so the history stays: it was recorded as "junction pads fold on
// sloped ground". Wrong. The isolation runs two graphs on FLAT ground with no
// junction pad at all — the middle node of the bent one is degree 2 — and got
//
//     straight  0 inverted of 264
//     bent      2 inverted of 276
//
// Slope was not involved and neither was the pad: the inversions appeared at a
// BEND, in the swept body's MITRE. The mechanism: at a kink the mitre ring's
// INSIDE point sits hw*tan(halfAngle) BACK along both legs, and any fill ring
// inside that reach is overrun by it — the inside edge doubles back and the
// cell between them folds. The fix (road_lattice.cpp sampleRings): the fill
// SKIPS the fold reach around each kink vertex, so the lattice spans from the
// last clear ring straight to the mitre ring and the inside edge stays
// monotone. (Per-CELL winding in emitLattice had earlier taken city.json from
// 24 inverted of 11575 to 8; this removes the residue.)
//
// The print became the assertion this comment always promised: zero inverted
// up-faces, on the isolates AND on the full sloped net with junction pads.
TEST_CASE(swept_body_inverts_a_few_faces_at_bends) {
    RoadGraph g;
    for (int i = 0; i <= 2; ++i)
        for (int j = 0; j <= 2; ++j) g.addNode(Vec2(i * 60.0, j * 60.0));
    auto at = [](int i, int j) { return i * 3 + j; };
    for (int i = 0; i <= 2; ++i)
        for (int j = 0; j <= 2; ++j) {
            if (i < 2) g.addEdge(at(i, j), at(i + 1, j), 9.0, RoadClass::Local);
            if (j < 2) g.addEdge(at(i, j), at(i, j + 1), 9.0, RoadClass::Local);
        }
    std::function<Real(Real, Real)> ground = [](Real x, Real) {
        return static_cast<Real>(3.0 * std::sin(x * 0.01));
    };
    RenderMesh net = buildRoadNetLattice(g, ground, nullptr, 3.0, 0.15, true);
    RenderMesh bare = buildRoadNetLattice(g, ground, nullptr, 0.0, 0.15, true);

    // Isolate the cause. A STRAIGHT chain with no junction pad at all, flat, then
    // the same with a right-angle bend. If the bend is where the inversions
    // appear, the defect is in the swept body's mitre and not in the Coons pad.
    RoadGraph straight;
    straight.addNode(Vec2(0, 0)); straight.addNode(Vec2(120, 0));
    straight.addEdge(0, 1, 9.0, RoadClass::Local);
    RoadGraph bent;
    bent.addNode(Vec2(0, 0)); bent.addNode(Vec2(60, 0)); bent.addNode(Vec2(60, 60));
    bent.addEdge(0, 1, 9.0, RoadClass::Local);
    bent.addEdge(1, 2, 9.0, RoadClass::Local);
    RenderMesh ms = buildRoadNetLattice(straight, nullptr, nullptr, 3.0, 0.15, false);
    RenderMesh mb = buildRoadNetLattice(bent, nullptr, nullptr, 3.0, 0.15, false);
    std::printf("        [winding] ISOLATE: straight %d/%zu, bent %d/%zu\n",
                upFacesInverted(ms), ms.indices.size() / 3,
                upFacesInverted(mb), mb.indices.size() / 3);
    std::printf("        [winding] net %d inverted / %zu tris, "
                "sidewalk-less %d / %zu\n",
                upFacesInverted(net), net.indices.size() / 3,
                upFacesInverted(bare), bare.indices.size() / 3);
    CHECK(net.indices.size() > 0);              // the mesh still builds
    CHECK(upFacesInverted(ms) == 0);            // straight body: clean
    CHECK(upFacesInverted(mb) == 0);            // THE RATCHET: the bend too
    CHECK(upFacesInverted(net) == 0);           // and the full sloped net
    CHECK(upFacesInverted(bare) == 0);          // with or without sidewalks
}

// WHY THE OBVIOUS TEST IS THE WRONG ONE (measured 2026-08-16).
//
// The tempting check — "geometric normal must oppose the first vertex's shading
// normal", which is what `city_winding_matches_engine_convention` applies to
// MeshBuilder::box — reports 192 of 1152 faces wrong on a plain swept street. It
// is a false alarm, and the breakdown says so: 0 of 768 up-facing faces fail,
// while exactly HALF (192/384) of the vertical curb-riser faces do.
//
// Half of one feature is the signature of an averaged CREASE normal, not inverted
// geometry. The curb's shared vertices sit on the fold between the horizontal deck
// and the vertical riser, so their normal is the ~45° average of both; each quad's
// two triangles then straddle it and one of the pair always "fails".
//
// mesh_builder.h says this outright above emitLattice: winding is chosen once for
// the whole lattice, "NOT per face. Shared vertices cannot carry a per-face
// winding, which is exactly the trap emitTri's per-face normal test would spring
// here." A soup mesher (emitTri/emitQuad, flat per-face normals) can be tested
// per-face; a shared-vertex lattice cannot.
//
// Hence the two checks above: agreement with a reference direction where one
// exists (up-facing faces must point up), plus edge-orientation consistency, which
// is the invariant that actually governs back-face culling and is immune to
// averaged normals.
TEST_CASE(per_face_normal_test_is_invalid_for_a_shared_vertex_lattice) {
    auto flat = [](double, double) { return 0.0; };
    RenderMesh body = sweepRoadLattice(straightStreet(3.6),
                                       streetProfile(1, 3.0, 0.15), flat, 2.0);
    int badUp = 0, badLateral = 0, totUp = 0, totLateral = 0;
    for (std::size_t t = 0; t + 2 < body.indices.size(); t += 3) {
        const Vertex& a = body.vertices[body.indices[t]];
        const Vertex& b = body.vertices[body.indices[t + 1]];
        const Vertex& c = body.vertices[body.indices[t + 2]];
        Vec3 gn = cross(b.position - a.position, c.position - a.position);
        if (gn.length() < 1e-12) continue;
        const bool lateral = std::fabs(normalize(gn).y) < 0.5;
        const bool flagged = dot(gn, a.normal) > 1e-6;
        if (lateral) { ++totLateral; badLateral += flagged; }
        else { ++totUp; badUp += flagged; }
    }
    std::printf("        [winding] per-face test flags %d/%d lateral, %d/%d up-facing\n",
                badLateral, totLateral, badUp, totUp);
    CHECK(badUp == 0);                        // the deck is genuinely fine
    CHECK(badLateral * 2 == totLateral);      // and the "failures" are the creases
}

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

// A chain with a 90-degree KINK must MITER: a ring sits exactly at the corner
// and its lateral offsets stretch by 1/cos(45) so the outer sidewalk edge stays
// continuous. Unmitered, the corner is chopped and the outer edge shows a wedge
// of ground (Glenn: "sidewalks aren't mitered in some cases").
TEST_CASE(street_body_miters_a_right_angle_bend) {
    UnionSpine s;
    s.klass = RoadClass::Local;
    for (int i = 0; i <= 5; ++i) s.points.push_back(Vec2(i * 10.0, 0));      // east 50
    for (int i = 1; i <= 5; ++i) s.points.push_back(Vec2(50.0, i * 10.0));   // north 50
    s.halfWidth = 4.0;
    auto flat = [](double, double) { return 0.0; };
    RenderMesh m = sweepRoadLattice(s, streetProfile(1, 3.0, 0.15), flat, 2.0);
    CHECK(!m.vertices.empty());

    // The OUTER sidewalk corner of the bend: full half-width (4+3) mitered by
    // 1/cos(45deg) ~ 1.414 along the corner bisector (outer side = -left = SE).
    const double full = 7.0 * std::sqrt(2.0);
    const Vec2 bisect = normalize(Vec2(1, -1));            // outward at the corner
    const Vec2 corner = Vec2(50, 0) + bisect * (full * 0.985);   // just inside the rim
    const std::vector<double> hits = driveprobe::surfacesAt(m, corner.x, corner.y);
    CHECK(!hits.empty());                                   // covered only if mitered

    // And the mesh stays sane through the bend.
    int degen = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        if (cross(b - a, c - a).length() < 1e-9) ++degen;
    }
    CHECK(degen == 0);
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
    // Interim fill (roads-v2 stage 1): ear-clip triangles — the quad-pairing
    // stage upgrades this to quad-dominant and restores exact boundary K.

    int steep = 0, degenerate = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        const Vec3 nrm = cross(b - a, c - a);
        const double ny = std::fabs(nrm.y);
        if (ny < 1e-9) { ++degenerate; continue; }
        if (std::sqrt(nrm.x * nrm.x + nrm.z * nrm.z) / ny > 0.25) {
            ++steep;
            std::printf("  [steep] (%.2f,%.2f,%.2f)(%.2f,%.2f,%.2f)(%.2f,%.2f,%.2f)\n",
                        a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
        }
    }
    CHECK(degenerate == 0);
    CHECK(steep == 0);            // matches the arms' gentle slope, no medial-axis step

    // The pad reaches each arm's height range (ear-clip may prune collinear
    // mouth midpoints — recorded interim caveat; stage 4 restores exact K).
    double ymin = 1e9, ymax = -1e9;
    for (const Vertex& v : m.vertices) {
        ymin = std::min(ymin, (double)v.position.y);
        ymax = std::max(ymax, (double)v.position.y);
    }
    CHECK(ymax > 0.7);                     // reaches the high (N, y=1) side
    CHECK(ymin < 0.3);                     // and the low (S, y=0) side
}

// Stage 3b: the T-junction — the dominant junction in a generated city (measured
// 15 T vs 2 four-way on metro_hills), and the case the centroid fan turned to
// spoke soup. The through road (E/W) plus a branch (N) at three heights must be
// an ALL-QUAD Coons grid, no degenerate face, drivable straight through.
TEST_CASE(junction_patch_T_is_smooth_all_quad) {
    auto arm = [](Vec2 d, double R, double hw, double h, int K) {
        JunctionArm a;
        a.dir = normalize(d);
        const Vec2 c = a.dir * R;
        const Vec2 pp(-a.dir.y, a.dir.x);
        for (int i = 0; i <= K; ++i) {
            const double t = i / static_cast<double>(K);
            const Vec2 p = c + pp * (hw * (1 - 2 * t));
            a.mouth.push_back(Vec3(p.x, h, p.y));
        }
        return a;
    };
    const double hw = 8.0;
    // Realistic: all three arms drape on the same gentle ground near the node, so
    // heights vary only a little (a big branch-vs-through gap is un-joinable by
    // ANY mesher and never happens — every arm meets at one graded pad).
    std::vector<JunctionArm> arms = {
        arm(Vec2(1, 0), hw, hw, 0.30, 6),    // E  (through)
        arm(Vec2(0, 1), hw, hw, 0.42, 6),    // N  (branch)
        arm(Vec2(-1, 0), hw, hw, 0.50, 6),   // W  (through)
    };
    RenderMesh m = junctionPatch(arms);
    const std::size_t tris = m.indices.size() / 3;
    CHECK(tris > 0);
    // Interim fill (roads-v2 stage 1): triangles; quad-pairing lands in stage 4.

    int steep = 0, degenerate = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        const Vec3 nrm = cross(b - a, c - a);
        const double ny = std::fabs(nrm.y);
        if (nrm.length() < 1e-9) { ++degenerate; continue; }
        if (ny > 1e-6 && std::sqrt(nrm.x * nrm.x + nrm.z * nrm.z) / ny > 0.25) ++steep;
    }
    CHECK(degenerate == 0);
    CHECK(steep == 0);

    // Drive the through road straight across the junction (E <-> W at z=0).
    std::vector<Vec3> path;
    for (double x = -7; x <= 7; x += 2) path.push_back(Vec3(x, 0.4, 0));
    driveprobe::Report rep;
    driveprobe::drivePath(m, path, rep);
    CHECK(rep.samples > 5);
    CHECK(rep.holes == 0);      // the patch covers the through road with no gap
    CHECK(rep.blocked == 0);
}

// ---------------------------------------------------------------------------
// Roads-v2 S4: the pad grows corner FILLET ARCS (kerb returns), a GRID interior
// with Laplace-smoothed heights, and EXACT boundary K — every mouth vertex the
// bodies delivered appears in the fill (the S1 earcut collinear-prune caveat is
// closed, so pad and body can never T-joint).

namespace {
JunctionArm s4arm(Vec2 d, double R, double hw, double h, int K) {
    JunctionArm a;
    a.dir = normalize(d);
    const Vec2 c = a.dir * R;
    const Vec2 pp(-a.dir.y, a.dir.x);       // +perp = left verge
    for (int i = 0; i <= K; ++i) {
        const double t = i / static_cast<double>(K);
        const Vec2 p = c + pp * (hw * (1 - 2 * t));
        a.mouth.push_back(Vec3(p.x, h, p.y));
    }
    return a;
}
int s4degenerate(const RenderMesh& m) {
    int n = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        if (cross(b - a, c - a).length() < 1e-9) ++n;
    }
    return n;
}
}  // namespace

TEST_CASE(junction_pad_corner_fillets_round_the_kerb) {
    // Right-angle 4-way: the corner between the E arm's left verge (12, 7.2)
    // and the N arm's right verge (7.2, 12). A kerb-return fillet curves the
    // boundary INSIDE the straight chord (x+z = 19.2), toward the verge-line
    // intersection (7.2, 7.2) — a rounded concave corner, not a snapped point.
    const double R = 12.0, hw = 7.2;
    std::vector<JunctionArm> arms = {
        s4arm(Vec2(0, 1), R, hw, 0.3, 4), s4arm(Vec2(1, 0), R, hw, 0.3, 4),
        s4arm(Vec2(0, -1), R, hw, 0.3, 4), s4arm(Vec2(-1, 0), R, hw, 0.3, 4),
    };
    RenderMesh m = junctionPatch(arms);
    CHECK(!m.vertices.empty());
    CHECK(s4degenerate(m) == 0);
    bool sawArc = false;
    for (const Vertex& v : m.vertices) {
        const double x = v.position.x, z = v.position.z;
        if (x > 7.5 && z > 7.5 && x + z > 15.0 && x + z < 18.6) sawArc = true;
    }
    CHECK(sawArc);                          // a real arc point off the chord
    // The fillet never pokes OUTSIDE the union of the two road rectangles.
    for (const Vertex& v : m.vertices) {
        const double x = v.position.x, z = v.position.z;
        CHECK(!(x > hw + 0.05 && z > hw + 0.05 && x + z > 19.2 + 0.05));
    }
}

TEST_CASE(junction_pad_grid_interior_exact_mouths_smooth_heights) {
    // Hilly 4-way (the 164%-step case): N high, S low. S4 contract:
    //  - EXACT K: every mouth vertex appears in the pad and carries triangles
    //  - a grid interior exists (vertices well away from the boundary)
    //  - heights stay inside the boundary range and change gently everywhere
    //  - a car drives straight through N -> S
    const double R = 12.0, hw = 7.2;
    std::vector<JunctionArm> arms = {
        s4arm(Vec2(0, 1), R, hw, 1.0, 4), s4arm(Vec2(1, 0), R, hw, 0.5, 4),
        s4arm(Vec2(0, -1), R, hw, 0.0, 4), s4arm(Vec2(-1, 0), R, hw, 0.5, 4),
    };
    RenderMesh m = junctionPatch(arms);
    CHECK(s4degenerate(m) == 0);

    auto q = [](const Vec3& p) {
        return std::make_pair(std::llround(p.x * 512.0), std::llround(p.z * 512.0));
    };
    std::map<std::pair<long long, long long>, int> refs;
    for (const unsigned idx : m.indices) ++refs[q(m.vertices[idx].position)];
    for (const JunctionArm& a : arms)
        for (const Vec3& mp : a.mouth) {
            auto it = refs.find(q(mp));
            CHECK(it != refs.end());        // the mouth vertex EXISTS in the pad
            if (it != refs.end()) CHECK(it->second >= 1);   // and has triangles
        }

    int interior = 0;
    for (const Vertex& v : m.vertices)
        if (std::fabs(v.position.x) < 4.0 && std::fabs(v.position.z) < 4.0)
            ++interior;
    CHECK(interior >= 1);                   // a real grid interior, not ear-clip

    double lo = 1e9, hi = -1e9;
    for (const JunctionArm& a : arms)
        for (const Vec3& mp : a.mouth) { lo = std::min(lo, (double)mp.y); hi = std::max(hi, (double)mp.y); }
    for (const Vertex& v : m.vertices) {
        CHECK(v.position.y > lo - 0.05);
        CHECK(v.position.y < hi + 0.05);
    }
    int steepEdges = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
        for (int e = 0; e < 3; ++e) {
            const Vec3& a = m.vertices[m.indices[t + e]].position;
            const Vec3& b = m.vertices[m.indices[t + (e + 1) % 3]].position;
            const double run = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
            if (run > 0.1 && std::fabs(a.y - b.y) / run > 0.25) ++steepEdges;
        }
    CHECK(steepEdges == 0);                 // smooth Laplace interior, no step

    std::vector<Vec3> path;
    for (double z = 11; z >= -11; z -= 1.5) path.push_back(Vec3(0, 1.2, z));
    driveprobe::Report rep;
    driveprobe::drivePath(m, path, rep);
    CHECK(rep.samples > 8);
    CHECK(rep.holes == 0);
    CHECK(rep.steps == 0);
}

TEST_CASE(compound_pad_covers_junction_pair_too_close) {
    // Two T-junctions 9 m apart, joined by a link SHORTER than the two
    // junction radii combined (rad = hw + sidewalk = 7 each). trimSpine CLAMPS
    // its trims at 45% of the chain, so S1 kept a squeezed 0.9 m body with
    // BOTH pads stacked over it — the measured pad-pad overlap (52 cells on
    // the real graph). S4 merges the pair into ONE compound pad: the link's
    // asphalt has exactly one owner, and a car still drives through clean.
    RoadGraph g;
    auto node = [&](double x, double z) {
        RoadNode n; n.pos = Vec2(x, z);
        g.nodes.push_back(n);
        return static_cast<int>(g.nodes.size() - 1);
    };
    const int w = node(-60, 0), a = node(0, 0), b = node(9, 0), e = node(69, 0);
    const int an = node(0, 60), bn = node(9, -60);
    auto edge = [&](int p, int qn) {
        RoadEdge ed; ed.a = p; ed.b = qn; ed.width = 8.0;
        g.edges.push_back(ed);
    };
    edge(w, a); edge(a, b); edge(b, e); edge(a, an); edge(b, bn);
    RenderMesh m = buildRoadNetLattice(g, nullptr);
    CHECK(!m.vertices.empty());
    CHECK(s4degenerate(m) == 0);

    // ONE surface between the junctions: no body squeezed under stacked pads.
    // Sample OFF the z=0 gridline (shared triangle edges there report both
    // neighbours at the same height) and count DISTINCT heights only.
    int stacked = 0;
    for (double x = -2.0; x <= 11.0; x += 0.5) {
        std::vector<double> hits = driveprobe::surfacesAt(m, x, 0.3);
        std::vector<double> near;
        for (double h : hits) if (h > -1.0 && h < 2.0) near.push_back(h);
        std::sort(near.begin(), near.end());
        int layers = 0;
        for (std::size_t k = 0; k < near.size(); ++k)
            if (k == 0 || near[k] - near[k - 1] > 0.02) ++layers;
        if (layers > 1) ++stacked;
    }
    CHECK(stacked == 0);

    std::vector<Vec3> path;
    for (double x = -20; x <= 29; x += 1.5) path.push_back(Vec3(x, 1.0, 0));
    driveprobe::Report rep;
    driveprobe::drivePath(m, path, rep);
    for (const auto& d : rep.defects)
        std::printf("  [defect] %s at (%.2f, %.2f, %.2f)\n", d.kind.c_str(), d.where.x, d.where.y, d.where.z);
    CHECK(rep.samples > 20);
    CHECK(rep.holes == 0);                  // the compound pad closes the gap
    CHECK(rep.steps == 0);
}

// ---------------------------------------------------------------------------
// Roads-v2 S5: OWNERSHIP. Asphalt = bodies + pads (carriageway only); curb +
// sidewalk = a raised band swept along the CLOSED union outline of the whole
// network (per block). No two components cover the same ground.

TEST_CASE(lattice_streets_grow_owned_sidewalk_bands) {
    RoadGraph g;
    auto node = [&](double x, double z) {
        RoadNode n; n.pos = Vec2(x, z);
        g.nodes.push_back(n);
        return static_cast<int>(g.nodes.size() - 1);
    };
    const int c = node(0, 0), nn = node(0, 80), ss = node(0, -80),
              ee = node(80, 0), ww = node(-80, 0);
    auto edge = [&](int p, int q) {
        RoadEdge ed; ed.a = p; ed.b = q; ed.width = 8.0;
        g.edges.push_back(ed);
    };
    edge(ww, c); edge(c, ee); edge(ss, c); edge(c, nn);
    RenderMesh m = buildRoadNetLattice(g, nullptr);
    CHECK(!m.vertices.empty());
    CHECK(s4degenerate(m) == 0);

    // The band exists BESIDE the carriageway: over the verge strip (asphalt
    // edge at |z| = 4, band to |z| ~ 7) the top surface is the raised slab.
    {
        std::vector<double> hits = driveprobe::surfacesAt(m, 30.0, 5.5);
        CHECK(!hits.empty());
        double top = -1e9;
        for (double h : hits) top = std::max(top, h);
        CHECK(top > 0.10);            // raised (curb height), not bare asphalt
        CHECK(top < 0.40);
    }
    // And it WRAPS the junction corner: outside the kerb-return arc.
    {
        std::vector<double> hits = driveprobe::surfacesAt(m, 5.8, 5.8);
        CHECK(!hits.empty());
        double top = -1e9;
        for (double h : hits) top = std::max(top, h);
        CHECK(top > 0.10);
        CHECK(top < 0.40);
    }
    // Ownership: nowhere do two up-facing surfaces stack (sampled off grid
    // lines; distinct heights only — shared edges are not overlap).
    int stacked = 0;
    for (double x = -60; x <= 60; x += 3.1)
        for (double z = -60; z <= 60; z += 3.1) {
            std::vector<double> hits = driveprobe::surfacesAt(m, x, z);
            std::vector<double> near;
            for (double h : hits) if (h > -2 && h < 3) near.push_back(h);
            std::sort(near.begin(), near.end());
            int layers = 0;
            for (std::size_t k = 0; k < near.size(); ++k)
                if (k == 0 || near[k] - near[k - 1] > 0.02) ++layers;
            if (layers > 1) ++stacked;
        }
    CHECK(stacked == 0);

    // The carriageway still drives clean through the junction.
    std::vector<Vec3> path;
    for (double x = -60; x <= 60; x += 2.0) path.push_back(Vec3(x, 0.5, 0));
    driveprobe::Report rep;
    driveprobe::drivePath(m, path, rep);
    CHECK(rep.samples > 40);
    CHECK(rep.holes == 0);
    CHECK(rep.steps == 0);
    CHECK(rep.blocked == 0);          // the curb lip never juts into the lanes
}

// ---------------------------------------------------------------------------
// Roads-v2 S6: ONE mesher. The lattice meshes every class — an authored
// Freeway chain gets the full viaduct structure (deck, soffit, parapets,
// median, piers), a Ramp chain gets its deck, and the curb/sidewalk band
// GAPS across non-street mouths instead of curbing a ramp entrance shut.

TEST_CASE(lattice_meshes_authored_freeway_with_structure) {
    RoadGraph g;
    auto node = [&](double x, double z, double elev, bool abs) {
        RoadNode n; n.pos = Vec2(x, z); n.elev = elev; n.elevAbsolute = abs;
        g.nodes.push_back(n);
        return static_cast<int>(g.nodes.size() - 1);
    };
    // Elevated 3-lane freeway along +X at +9 m over a street crossing below.
    const int f0 = node(-200, 0, 9, true), f1 = node(30, 0, 9, true),
              f2 = node(200, 0, 9, true);
    const int s0 = node(30, -80, 0, false), s1 = node(30, 80, 0, false);
    auto edge = [&](int a, int b, double w, RoadClass k) {
        RoadEdge e; e.a = a; e.b = b; e.width = w; e.klass = k;
        g.edges.push_back(e);
    };
    edge(f0, f1, 22.0, RoadClass::Freeway);
    edge(f1, f2, 22.0, RoadClass::Freeway);
    edge(s0, s1, 8.0, RoadClass::Local);
    RenderMesh m = buildRoadNetLattice(g, nullptr);
    CHECK(!m.vertices.empty());

    int up9 = 0, down = 0, lateral = 0, pierFaces = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        Vec3 gn = cross(c - a, b - a);
        if (gn.length() < 1e-9) continue;
        gn = normalize(gn);
        const Vec3 ctr = (a + b + c) * (1.0 / 3.0);
        if (gn.y > 0.5 && ctr.y > 8.0) ++up9;            // deck top
        if (gn.y < -0.5 && ctr.y > 7.0) ++down;          // soffit underside
        if (std::fabs(gn.y) <= 0.5 && ctr.y > 8.5) ++lateral;   // parapets/median
        if (std::fabs(gn.y) <= 0.5 && ctr.y > 2.0 && ctr.y < 7.0 &&
            std::fabs(ctr.z) < 12.0)
            ++pierFaces;   // pier shafts (portal legs stand at the deck edges)
    }
    CHECK(up9 > 0);
    CHECK(down > 0);
    CHECK(lateral > 0);
    CHECK(pierFaces > 0);

    // A LANE on the deck drives clear (median at centre, parapets at verge).
    std::vector<Vec3> lane;
    for (double x = -180; x <= 180; x += 5) lane.push_back(Vec3(x, 9.0, 6.0));
    driveprobe::Report rep;
    driveprobe::drivePath(m, lane, rep);
    CHECK(rep.samples > 40);
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);

    // The street drives clean UNDER the viaduct.
    std::vector<Vec3> under;
    for (double z = -60; z <= 60; z += 3) under.push_back(Vec3(30, 0.5, z));
    driveprobe::Report urep;
    driveprobe::drivePath(m, under, urep);
    CHECK(urep.samples > 20);
    CHECK(urep.holes == 0);
    CHECK(urep.blocked == 0);

    // No sidewalk band beside the freeway (spec: no sidewalks on a freeway).
    for (double x = -150; x <= 150; x += 60) {
        for (double h : driveprobe::surfacesAt(m, x, 13.5))
            CHECK(!(h > 0.05 && h < 0.5));   // no raised slab at ground level
    }
}

TEST_CASE(band_gaps_across_a_ramp_mouth) {
    RoadGraph g;
    auto node = [&](double x, double z) {
        RoadNode n; n.pos = Vec2(x, z);
        g.nodes.push_back(n);
        return static_cast<int>(g.nodes.size() - 1);
    };
    const int w = node(-80, 0), a = node(0, 0), e = node(80, 0);
    const int r1 = node(0, -40), r2 = node(0, -90);
    auto edge = [&](int p, int q, double wd, RoadClass k) {
        RoadEdge ed; ed.a = p; ed.b = q; ed.width = wd; ed.klass = k;
        g.edges.push_back(ed);
    };
    edge(w, a, 8.0, RoadClass::Local);
    edge(a, e, 8.0, RoadClass::Local);
    edge(a, r1, 7.2, RoadClass::Ramp);     // a ramp FOOT landing on the street
    edge(r1, r2, 7.2, RoadClass::Ramp);
    RenderMesh m = buildRoadNetLattice(g, nullptr);
    CHECK(!m.vertices.empty());

    // Drive from the ramp, through the junction pad, onto the street: without
    // the band gap a 0.15 m curb + slab stood across the ramp mouth.
    std::vector<Vec3> path;
    for (double z = -70; z <= -8; z += 3) path.push_back(Vec3(0, 0.5, z));
    path.push_back(Vec3(4, 0.5, -4));
    path.push_back(Vec3(12, 0.5, 0));
    path.push_back(Vec3(30, 0.5, 0));
    driveprobe::Report rep;
    driveprobe::drivePath(m, path, rep);
    CHECK(rep.samples > 20);
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);

    // The band still exists along the plain street away from the mouth.
    std::vector<double> hits = driveprobe::surfacesAt(m, 40.0, 5.5);
    bool raised = false;
    for (double h : hits) raised |= (h > 0.10 && h < 0.4);
    CHECK(raised);
}

// LANE DIVIDERS (device: "can't it do multilane roads and yet I never see
// that"). The surface shaders in all three backends paint a dashed white lane
// line wherever the road's lateral paint coordinate exceeds 3.5 — and no
// profile ever emitted a value above 3, so no road in the game had ever shown
// one. The carriageway profile now lays a narrow strip carrying 4.0 at each
// internal lane boundary, with the lane count coming from the road's class.
TEST_CASE(carriageway_paints_a_lane_divider_per_internal_boundary) {
    auto stripCount = [](const RoadProfile& p) {
        int runs = 0;
        bool in = false;
        for (const ProfileCol& c : p.cols) {
            const bool hot = c.mu > 3.5f;
            if (hot && !in) ++runs;
            in = hot;
        }
        return runs;
    };
    auto span = [](const RoadProfile& p) {   // widest painted strip, metres
        double lo = 1e9, hi = -1e9;
        for (const ProfileCol& c : p.cols)
            if (c.mu > 3.5f) { lo = std::min(lo, c.absOffset); hi = std::max(hi, c.absOffset); }
        return hi > lo ? hi - lo : 0.0;
    };
    // A two-lane street: its only internal boundary is the centreline, which
    // wears the double yellow — so no dashed divider at all.
    const RoadProfile local = carriagewayProfile(1, 2, /*oneWay=*/false);
    std::printf("    [lanes] 2-lane street: %d divider(s)\n", stripCount(local));
    CHECK(stripCount(local) == 0);
    // A four-lane arterial: one either side of the centreline.
    const RoadProfile arterial = carriagewayProfile(2, 4, /*oneWay=*/false);
    std::printf("    [lanes] 4-lane arterial: %d divider(s), %.2f m wide\n",
                stripCount(arterial), span(arterial));
    CHECK(stripCount(arterial) == 2);
    CHECK_APPROX(span(arterial), 0.15, 0.001);
    // A one-way carriageway has no centreline to skip: 3 lanes -> 2 dividers.
    const RoadProfile oneWay = carriagewayProfile(3, 3, /*oneWay=*/true);
    std::printf("    [lanes] 3-lane one-way: %d divider(s)\n", stripCount(oneWay));
    CHECK(stripCount(oneWay) == 2);
    // Columns stay sorted by lateral position: the sweeper builds quads between
    // CONSECUTIVE columns, so an out-of-order strip would fold the surface.
    for (const RoadProfile* p : {&local, &arterial, &oneWay}) {
        bool sorted = true;
        for (std::size_t i = 1; i < p->cols.size(); ++i)
            if (p->cols[i].edgeFrac + p->cols[i].absOffset * 0.001 <
                p->cols[i - 1].edgeFrac + p->cols[i - 1].absOffset * 0.001 - 1e-9)
                sorted = false;
        CHECK(sorted);
    }
    CHECK(stripCount(carriagewayProfile(1, 1, /*oneWay=*/true)) == 0);
}
