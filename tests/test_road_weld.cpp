#include "test_framework.h"
#include "mesh_invariants.h"

#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/road_offset.h"

#include <limits>

using namespace engine;
using namespace mesh_invariants;

namespace {

// A degree-N junction: a centre node at the origin with `dirs` arms radiating to
// `armLen`. edgeWidths/edgeClasses parallel the arms (empty = default).
RoadNet junction(const std::vector<Vec2>& dirs, double armLen,
                 std::vector<double> widths = {},
                 std::vector<RoadClass> classes = {}) {
    RoadNet n;
    n.nodes.push_back(Vec2(0, 0));                 // centre = node 0
    for (std::size_t i = 0; i < dirs.size(); ++i) {
        Vec2 d = dirs[i];
        double l = d.length();
        Vec2 u = l > 1e-9 ? d * (1.0 / l) : Vec2(1, 0);
        n.nodes.push_back(u * armLen);
        n.edges.push_back({0, static_cast<int>(i + 1)});
    }
    n.edgeWidths = std::move(widths);
    n.edgeClasses = std::move(classes);
    return n;
}

// Assert the universal weld invariants on a fixture's meshed output.
void checkWeld(const RoadNet& net, const char* /*label*/) {
    RenderMesh m = buildRoadNetMesh(net);
    CHECK(triangleCount(m) > 0);                   // something was welded
    CHECK(!hasNonFinite(m));                       // no NaN/Inf blow-up
    CHECK(indicesInRange(m));                      // every index is real
    CHECK(degenerateTriangles(m) == 0);            // no zero-area / dup-index tris
    // A real up-facing deck exists (not flipped). Valid junctions run ~0.37-0.71
    // up (acute corners add sidewall, lowering it); a flipped/missing deck
    // collapses toward ~0.15, which this catches.
    CHECK(upwardFraction(m) > 0.3);
    // vertices stay within a sane box around the ~90 m fixture (no vertex flung off)
    double minX, maxX, minZ, maxZ;
    bboxXZ(m, minX, maxX, minZ, maxZ);
    CHECK(minX > -200.0 && maxX < 200.0);
    CHECK(minZ > -200.0 && maxZ < 200.0);
}

}  // namespace

// A — a straight through road (degree-2, one chain).
TEST_CASE(weld_straight_chain_is_clean) {
    RoadNet n;
    n.nodes = { Vec2(-40, 0), Vec2(40, 0) };
    n.edges = { {0, 1} };
    checkWeld(n, "straight");
}

// B — a T-junction (degree-3).
TEST_CASE(weld_tee_is_clean) {
    checkWeld(junction({Vec2(-1, 0), Vec2(1, 0), Vec2(0, 1)}, 40.0), "tee");
}

// C — a 4-way cross (degree-4).
TEST_CASE(weld_cross_is_clean) {
    checkWeld(junction({Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)}, 40.0), "cross");
}

// D — a skewed cross (arms at odd angles) — the acute-corner stress case.
TEST_CASE(weld_skewed_cross_is_clean) {
    checkWeld(junction({Vec2(-1, 0.3), Vec2(1, 0.1), Vec2(-0.2, -1), Vec2(0.4, 1)}, 40.0),
              "skewed");
}

// E — WIDE meets NARROW: a wide arterial through-road crossed by a narrow local.
// The mixed-width junction that drove the crosswalk-jut; assert its weld is clean.
TEST_CASE(weld_wide_meets_narrow_is_clean) {
    RoadNet n = junction({Vec2(-1, 0), Vec2(1, 0), Vec2(0, 1)}, 40.0,
                         /*widths*/ {13.0, 13.0, 7.0},
                         /*classes*/ {RoadClass::Arterial, RoadClass::Arterial,
                                      RoadClass::Local});
    checkWeld(n, "wide_meets_narrow");
}

// F — a very ACUTE Y: two arms nearly parallel (~15 deg apart). The audit flags
// acute corners as the sidewalk-tear / ground-wedge case; assert the weld stays
// geometrically clean here too.
TEST_CASE(weld_acute_y_is_clean) {
    checkWeld(junction({Vec2(1, 0), Vec2(1, 0.27), Vec2(-1, 0.05)}, 40.0), "acute_y");
}

// G — a shallow-angle T: the stem meets the through road at ~20 deg (a slip road
// tee-ing into a street), the classic acute-junction stress.
TEST_CASE(weld_shallow_tee_is_clean) {
    checkWeld(junction({Vec2(-1, 0), Vec2(1, 0), Vec2(0.94, 0.34)}, 40.0), "shallow_tee");
}

// H — CROSSWALK-JUT regression: a narrow local (hw 3) crossing a WIDE arterial
// (hw 7) at a SHALLOW angle must set its zebra back past the arterial's OBLIQUE
// ribbon (mouth = hw/sin θ), not by the ~7 m disc radius (which would jut onto
// the arterial). The setback is baked into the road-local v UV (v = distToEnd −
// mouth), so the crosswalk band (v ≈ 0) on the stub starts at world distance
// ≈ mouth from the node — measure where it lands.
namespace {
double stubBandDistance(const std::vector<UnionSpine>& sp) {
    WeldSolidParams p; p.crosswalks = true;
    RenderMesh m = weldSolid(sp, p);
    const Vec2 raw = sp[2].points.back() - sp[2].points.front();
    const Vec2 dir = raw * (1.0 / raw.length());       // stub heading from the node
    double band = 1e9;
    for (const Vertex& vx : m.vertices) {
        if (vx.u <= 0.5) continue;                     // painted strip only
        const Vec2 q(vx.position.x, vx.position.z);
        const double along = q.x * dir.x + q.y * dir.y;
        const double lat = std::fabs(q.x * -dir.y + q.y * dir.x);
        if (along < 2.0 || lat > 4.0) continue;        // on the stub, clear of the node
        if (std::fabs(static_cast<double>(vx.v)) < 0.6) band = std::min(band, along);
    }
    return band;   // world distance from node to the stub's crosswalk band
}
std::vector<UnionSpine> tJunction(Vec2 stub, double armHW, double stubHW) {
    auto arm = [](Vec2 a, Vec2 b, double hw, RoadClass k) {
        UnionSpine s; s.halfWidth = hw; s.klass = k;
        for (int i = 0; i <= 24; ++i) s.points.push_back(a + (b - a) * (i / 24.0));
        return s;
    };
    const Vec2 stubEnd = stub * (72.0 / stub.length());
    return { arm(Vec2(-70, 0), Vec2(0, 0), armHW, RoadClass::Arterial),
             arm(Vec2(0, 0), Vec2(70, 0), armHW, RoadClass::Arterial),
             arm(Vec2(0, 0), stubEnd, stubHW, RoadClass::Local) };
}
}  // namespace
TEST_CASE(weld_crosswalk_clears_skewed_arm) {
    // Oblique local meets the wide arterial at ~26°: mouth = 7/sin(26°) ≈ 16 m.
    const double band = stubBandDistance(tJunction(Vec2(0.90, 0.44), 7.0, 3.0));
    CHECK(band > 12.0 && band < 22.0);   // skew-aware setback, NOT the ~7 m disc radius
}

// --- 3-D channel (welder-goes-3D): an ELEVATED deck rides its authored
// absolute Y through the SAME welder that meshes streets. This is the unit-level
// proof that one mesher can carry a corridor/ramp — the fork's whole reason for
// existing (a single-valued drape can't lift a deck) is what UnionSpine.yAbs and
// weldChainProfiles' authored-height path retire.

// H — weldChainProfiles hands back the authored deck Y UNTOUCHED: not draped to
// terrain, not pulled to a junction min, not offset by topY. A spine at a
// constant +9 m must come back at +9 m regardless of the ground beneath it.
TEST_CASE(weld_authored_deck_rides_its_own_height) {
    UnionSpine s;
    s.points = { Vec2(-40, 0), Vec2(0, 0), Vec2(40, 0) };
    s.halfWidth = 6.0;
    s.klass = RoadClass::Freeway;
    s.yAbs = { 9.0, 9.0, 9.0 };                    // absolute deck Y
    // A sloped ground far BELOW the deck: drape would put the road here, not +9.
    auto ground = [](double x, double) { return -20.0 + 0.05 * x; };
    auto profs = weldChainProfiles({ s }, ground, /*topY*/ 0.06, /*maxGrade*/ 0.08);
    CHECK(profs.size() == 1);
    CHECK(profs[0].size() == 3);
    for (double h : profs[0]) CHECK(std::fabs(h - 9.0) < 1e-6);   // rode yAbs exactly
}

// I — a draped spine in the SAME batch still drapes: the authored-height path is
// per-spine, so mixing an elevated deck with an at-grade street leaves the street
// on the ground. Proves the 3-D path is additive, not a global mode switch.
TEST_CASE(weld_authored_and_draped_spines_coexist) {
    UnionSpine deck;
    deck.points = { Vec2(-40, 0), Vec2(40, 0) };
    deck.halfWidth = 6.0;
    deck.yAbs = { 9.0, 9.0 };
    UnionSpine street;                             // no yAbs -> drapes
    street.points = { Vec2(0, -40), Vec2(0, 40) };
    street.halfWidth = 4.0;
    auto ground = [](double, double) { return 2.5; };
    auto profs = weldChainProfiles({ deck, street }, ground, 0.06, 0.08);
    CHECK(profs.size() == 2);
    for (double h : profs[0]) CHECK(std::fabs(h - 9.0) < 1e-6);   // deck: authored
    for (double h : profs[1]) CHECK(std::fabs(h - (2.5 + 0.06)) < 1e-6);  // street: drape+topY
}

// J — weldSolid MESHES the elevated deck: the same solid welder that builds
// streets seats the deck at the authored +9 and carries it on piers down to the
// ground reference (topY when no terrain is given). Deck top aloft, piers reach
// down — not draped to grade, not a solid curtain wall from deck to ground.
TEST_CASE(weld_solid_meshes_elevated_deck) {
    UnionSpine s;
    s.points = { Vec2(-40, 0), Vec2(0, 0), Vec2(40, 0) };
    s.halfWidth = 6.0;
    s.klass = RoadClass::Freeway;
    s.yAbs = { 9.0, 9.0, 9.0 };
    WeldSolidParams p;
    p.thickness = 0.5; p.topY = 0.06;
    RenderMesh m = weldSolid({ s }, p);
    CHECK(triangleCount(m) > 0);
    CHECK(!hasNonFinite(m));
    CHECK(indicesInRange(m));
    CHECK(degenerateTriangles(m) == 0);
    CHECK(upwardFraction(m) > 0.3);
    double minY, maxY;
    bboxY(m, minY, maxY);
    CHECK(maxY > 8.0 && maxY < 10.0);              // deck rode to the authored +9
    CHECK(minY < 1.0);                             // piers reach down to the ground ref
}

// M — PIERS hold up the deck and AVOID the road below. An elevated deck runs
// along Z; a street runs along X and passes under it at the origin. Piers march
// down the deck to the ground EXCEPT where the street passes beneath — the
// obstruction-clearance rule (no column dropped onto the carriageway).
TEST_CASE(weld_solid_piers_hold_deck_and_avoid_road) {
    UnionSpine deck;                               // along Z, elevated to +9
    deck.points = { Vec2(0, -40), Vec2(0, 40) };
    deck.halfWidth = 6.0;
    deck.klass = RoadClass::Freeway;
    deck.yAbs = { 9.0, 9.0 };
    UnionSpine street;                             // along X, at grade, halfWidth 5
    street.points = { Vec2(-40, 0), Vec2(40, 0) };
    street.halfWidth = 5.0;
    WeldSolidParams p;
    p.thickness = 0.5;
    p.heightAt = [](double, double) { return 0.0; };   // flat ground at y=0
    RenderMesh m = weldSolid({ deck, street }, p);
    CHECK(!hasNonFinite(m));
    CHECK(indicesInRange(m));
    CHECK(degenerateTriangles(m) == 0);
    // A pier is a box from the deck underside (~8.5) down to the ground (y=0);
    // its BOTTOM vertices sit exactly on the ground plane. Nothing else lands
    // there: the street deck rides at ~+0.06 and its underside at ~-0.5, the
    // deck slab is at ~8.5/9. So |y|<0.02 on the deck centreline (|x|<1.2)
    // isolates pier feet. z=-4 is over the street; z=-22/+14/+32 are clear.
    int pierAwayFromRoad = 0, pierOverRoad = 0;
    for (const Vertex& v : m.vertices) {
        const double y = v.position.y, x = v.position.x, z = v.position.z;
        if (std::fabs(y) > 0.02 || std::fabs(x) > 1.2) continue;   // not a pier foot
        if (std::fabs(z) < 5.0) ++pierOverRoad;    // within the street corridor
        else if (std::fabs(z) > 10.0) ++pierAwayFromRoad;
    }
    CHECK(pierOverRoad == 0);                       // no column dropped on the street
    CHECK(pierAwayFromRoad > 0);                    // but the deck IS held up elsewhere
}

// K — the WHOLE authoring path: a RoadNet with per-node absolute elevation
// (nodeElev) is meshed by buildRoadNetMesh — the SAME entry point streets use —
// into an elevated deck. Proves an author (or a future corridor publisher) can
// express a 3-D road as graph data and the one welder builds it; no separate
// bridge mesher, no layer>0 bail.
TEST_CASE(road_net_meshes_an_authored_elevated_span) {
    RoadNet n;
    n.nodes = { Vec2(-40, 0), Vec2(40, 0) };
    n.edges = { {0, 1} };
    n.nodeElev = { 9.0, 9.0 };                     // both ends at +9 -> elevated span
    n.heightAt = [](double, double) { return -12.0; };   // ground far below
    RenderMesh m = buildRoadNetMesh(n);
    CHECK(triangleCount(m) > 0);
    CHECK(!hasNonFinite(m));
    CHECK(indicesInRange(m));
    CHECK(degenerateTriangles(m) == 0);
    CHECK(upwardFraction(m) > 0.3);
    double minY, maxY;
    bboxY(m, minY, maxY);
    CHECK(maxY > 8.0 && maxY < 10.0);              // deck rode to the authored +9
    CHECK(minY < -8.0);                            // piers reach down toward the -12 ground
}

// P1 (one mesher) — a FREEWAY-class deck gets divided-highway BARRIERS from the
// SAME welder that meshes streets: edge parapets + a solid median box standing
// above the deck. A Local street of the identical shape gets NONE. This is the
// weld gaining a freeway feature that used to live only in corridor_mesh.
TEST_CASE(weld_freeway_deck_gets_barriers) {
    auto makeNet = [](RoadClass k) {
        RoadNet n;
        for (int i = 0; i <= 8; ++i) n.nodes.push_back(Vec2(-140 + i * 35, 0));
        n.nodeElev = { 0, 3, 7, 9, 9, 9, 7, 3, 0 };    // elevated span, +9 in the middle
        for (int i = 0; i < 8; ++i) { n.edges.push_back({i, i + 1}); n.edgeClasses.push_back(k); }
        n.width = 22.0; n.sidewalk = 0.0; n.markings = true; n.autoRoundabout = false;
        return n;
    };
    RenderMesh fw = buildRoadNetMesh(makeNet(RoadClass::Freeway));
    RenderMesh st = buildRoadNetMesh(makeNet(RoadClass::Local));
    CHECK(!hasNonFinite(fw));
    CHECK(degenerateTriangles(fw) == 0);
    double fwMinY, fwMaxY, stMinY, stMaxY;
    bboxY(fw, fwMinY, fwMaxY); bboxY(st, stMinY, stMaxY);
    // Count vertices standing clearly above the deck: barrier walls on the
    // freeway, none on the street of the same shape.
    auto above = [](const RenderMesh& m, double y) {
        int c = 0; for (const Vertex& v : m.vertices) if (v.position.y > y) ++c; return c;
    };
    CHECK(fwMaxY > 9.3);
    CHECK(stMaxY < 9.3);                                // a Local deck tops out at the asphalt (+9)
    CHECK(above(fw, 9.3) > 0);
    CHECK(above(st, 9.3) == 0);
}

// P3 (one mesher) — the CorridorDef->spines ADAPTER meshes a straight elevated
// freeway through weldSolid to PARITY with buildCorridorMesh: same deck-top
// height, same width, same length. Proves the corridor's authoring core
// (alignment + vertical profile + lane widths) can drive the UNIFIED welder
// instead of corridor_mesh's own geometry pass.
TEST_CASE(corridor_deck_spines_match_the_corridor_mesher) {
    CorridorDef c;
    c.horizontal = Alignment::fromPolyline({ Vec2(-100, 0), Vec2(100, 0) }, 300.0, 20.0);
    c.vertical.pvis = { {0.0, 9.0, 0.0}, {c.horizontal.length(), 9.0, 0.0} };  // flat, +9 elevated
    c.lanes.throughLanes = 3;
    auto ground = [](Real, Real) { return 0.0; };
    CorridorMeshOut cm = buildCorridorMesh(c, ground, 3.0, nullptr);
    std::vector<UnionSpine> sp = corridorDeckSpines(c, ground, 3.0);
    CHECK(sp.size() == 1);
    WeldSolidParams wp; wp.barriers = false;                 // compare the bare drivable slab
    RenderMesh weld = weldSolid(sp, wp);
    CHECK(!weld.vertices.empty());
    CHECK(!hasNonFinite(weld));
    double cmMinY, cmMaxY, wMinY, wMaxY;
    bboxY(cm.deck, cmMinY, cmMaxY); bboxY(weld, wMinY, wMaxY);
    CHECK(cmMaxY > 8.5 && cmMaxY < 9.5);                     // corridor deck tops at +9
    CHECK(std::fabs(wMaxY - cmMaxY) < 0.6);                  // weld deck matches
    double cx0, cx1, cz0, cz1, wx0, wx1, wz0, wz1;
    bboxXZ(cm.deck, cx0, cx1, cz0, cz1); bboxXZ(weld, wx0, wx1, wz0, wz1);
    CHECK(std::fabs((cx1 - cx0) - (wx1 - wx0)) < 8.0);       // same length (along X)
    CHECK(std::fabs((cz1 - cz0) - (wz1 - wz0)) < 4.0);       // same deck width (across Z)
}

// P6 (one mesher) — a CURVED corridor banks: corridorDeckSpines pulls the
// alignment's superelevation into the deck spine's crossSlope, so the SAME
// banking the corridor mesher applied is now driven through the weld. A curve
// tight enough to superelevate must yield a non-zero cross-slope somewhere.
TEST_CASE(curved_corridor_deck_spine_carries_superelevation) {
    CorridorDef c;
    c.horizontal = Alignment::fromPolyline(
        { Vec2(-200, 0), Vec2(0, 0), Vec2(120, 120), Vec2(120, 320) }, 180.0, 40.0);
    c.vertical.pvis = { {0.0, 9.0, 0.0}, {c.horizontal.length(), 9.0, 0.0} };
    c.lanes.throughLanes = 3;
    auto ground = [](Real, Real) { return 0.0; };
    std::vector<UnionSpine> sp = corridorDeckSpines(c, ground, 3.0);
    CHECK(sp.size() == 1);
    CHECK(sp[0].crossSlope.size() == sp[0].points.size());   // banking channel populated
    double maxBank = 0.0;
    for (double cs : sp[0].crossSlope) maxBank = std::max(maxBank, std::fabs(cs));
    CHECK(maxBank > 0.01);   // the curve superelevates — deck tilts through the bend

    // ...and the weld honours it: the banked deck is no longer a flat plane.
    WeldSolidParams wp; wp.barriers = false;
    RenderMesh m = weldSolid(sp, wp);
    CHECK(!m.vertices.empty());
    CHECK(!hasNonFinite(m));
    double minY, maxY; bboxY(m, minY, maxY);
    CHECK(maxY - minY > 0.05);   // a flat +9 deck would be a razor slab; banking gives it relief
}

// P4 (one mesher) — a corridor RAMP centreline (rampPaths, the same authored
// polyline the nav graph reads) becomes a ramp UnionSpine the ONE welder meshes:
// it rides the deck at the top (+9) and the grade-sep split descends its low
// foot to street grade (0). Because weld AND nav both read rampPaths, they can't
// disagree — the truth-source inversion never happens.
TEST_CASE(corridor_ramp_spine_descends_from_deck_to_grade) {
    CorridorMeshOut::RampPath rp;
    for (int i = 0; i <= 12; ++i) {                    // deck +9 -> street 0, curving away
        const double t = i / 12.0;
        rp.pts.push_back(Vec3(t * 130.0, 9.0 * (1.0 - t), 24.0 * t * t));
    }
    std::vector<UnionSpine> sp = corridorRampSpines({ rp }, 3.6);
    CHECK(sp.size() == 1);
    CHECK(sp[0].klass == RoadClass::Ramp);
    CHECK(sp[0].yAbs.size() == rp.pts.size());
    auto ground = [](Real, Real) { return 0.0; };
    WeldSolidParams wp; wp.heightAt = ground;
    RenderMesh m = weldSolid(sp, wp);
    CHECK(!m.vertices.empty());
    CHECK(!hasNonFinite(m));
    CHECK(degenerateTriangles(m) == 0);
    double minY, maxY;
    bboxY(m, minY, maxY);
    CHECK(maxY > 8.0);   // rides the deck (+9) at the top
    CHECK(minY < 1.0);   // welds down to street grade (0) at the bottom
}

// P5 (one mesher) — VARIABLE-WIDTH ribbon outline: a centreline whose per-vertex
// half-width grows produces a FLARING ribbon (the aux-lane / gore widening). The
// hard piece that lets a freeway deck diverge instead of running one width.
TEST_CASE(variable_ribbon_outline_flares) {
    std::vector<Vec2> cl = { Vec2(0, 0), Vec2(30, 0), Vec2(60, 0) };
    std::vector<double> hw = { 3.0, 3.0, 8.0 };            // widens toward the end
    Poly2 out = ribbonOutline(cl, hw);
    CHECK(out.size() >= 4);
    double maxLat = 0.0, startLat = 0.0;
    for (const Vec2& p : out) {
        maxLat = std::max(maxLat, std::fabs(p.y));
        if (std::fabs(p.x) < 1.0) startLat = std::max(startLat, std::fabs(p.y));
    }
    CHECK(maxLat > 7.0 && maxLat < 9.0);      // flared to the hw 8 end
    CHECK(startLat > 2.0 && startLat < 4.0);  // still hw 3 at the start
}

// The weld consumes the per-point width: an elevated deck spine whose hw ramps
// 3 -> 8 comes out WIDER at the wide end. Empty hw is unchanged (all other tests).
TEST_CASE(weld_deck_flares_with_per_point_width) {
    UnionSpine s;
    s.klass = RoadClass::Freeway;
    for (int i = 0; i <= 10; ++i) {
        s.points.push_back(Vec2(-50 + i * 10.0, 0));
        s.hw.push_back(3.0 + 5.0 * (i / 10.0));           // 3 -> 8
        s.yAbs.push_back(9.0);                            // authored elevated deck
    }
    s.halfWidth = 8.0;
    WeldSolidParams wp;
    wp.barriers = false;
    wp.heightAt = [](Real, Real) { return 0.0; };
    RenderMesh m = weldSolid({ s }, wp);
    CHECK(!m.vertices.empty());
    CHECK(!hasNonFinite(m));
    double zNarrow = 0.0, zWide = 0.0;
    for (const Vertex& v : m.vertices) {
        if (v.position.y < 8.98 || v.position.y > 9.02) continue;   // deck top (excl. markings at 9.03)
        if (v.position.x < -40) zNarrow = std::max(zNarrow, std::fabs((double)v.position.z));
        if (v.position.x >  40) zWide   = std::max(zWide,   std::fabs((double)v.position.z));
    }
    CHECK(zNarrow > 2.0 && zNarrow < 4.5);   // ~hw 3 at the narrow end
    CHECK(zWide  > 6.0 && zWide  < 9.5);     // ~hw 8 — the deck flared
}

// P6 (one mesher) — SUPERELEVATION: a deck with a cross-slope BANKS, its left
// edge riding higher than its right (the corridor's curve banking, now in the
// weld). Empty crossSlope is flat (all other decks unchanged).
TEST_CASE(weld_deck_banks_with_cross_slope) {
    UnionSpine s;
    s.klass = RoadClass::Freeway;
    for (int i = 0; i <= 8; ++i) {
        s.points.push_back(Vec2(-40 + i * 10.0, 0));
        s.yAbs.push_back(9.0);                 // flat centreline at +9
        s.crossSlope.push_back(0.06);          // 6% cross-slope, rises to the LEFT
    }
    s.halfWidth = 8.0;
    WeldSolidParams wp;
    wp.barriers = false;
    wp.heightAt = [](Real, Real) { return 0.0; };
    RenderMesh m = weldSolid({ s }, wp);
    CHECK(!m.vertices.empty());
    CHECK(!hasNonFinite(m));
    double yLeft = -1e9, yRight = -1e9;         // max deck Y at each edge
    for (const Vertex& v : m.vertices) {
        if (v.position.z > 6.0) yLeft = std::max(yLeft, (double)v.position.y);
        if (v.position.z < -6.0) yRight = std::max(yRight, (double)v.position.y);
    }
    // At |z| = 8, banking = 8 * 0.06 = 0.48: left edge ~9.48, right edge ~8.52.
    CHECK(yLeft > 9.2);    // left edge banked UP
    CHECK(yRight < 8.9);   // right edge banked DOWN
}

// L — a NaN/short nodeElev leaves the road at grade: authoring is opt-in and
// per-node, so an ordinary street with no elevation drapes exactly as before.
TEST_CASE(road_net_without_node_elev_stays_at_grade) {
    RoadNet n;
    n.nodes = { Vec2(-40, 0), Vec2(40, 0) };
    n.edges = { {0, 1} };
    n.heightAt = [](double, double) { return 3.0; };
    RenderMesh m = buildRoadNetMesh(n);
    double minY, maxY;
    bboxY(m, minY, maxY);
    CHECK(maxY < 4.0);                             // near the 3.0 ground (+ lift), not lifted
}

// --- Grade separation (P4): a deck flies OVER a live road ---------------------
// The 2-D weld used to fuse a deck and the street it crosses into one surface +
// a curtain wall to the ground. weldSolid now welds each grade group separately,
// so the deck and the street beneath it are two independent surfaces with clear
// air between. Height-band assertions are robust to the thin marking overlay.

namespace {
// A constant +9 m deck (along Z) crossing an at-grade street (along X) at origin.
RenderMesh flyover() {
    UnionSpine deck;
    deck.points = { Vec2(0, -40), Vec2(0, 40) };
    deck.halfWidth = 6.0; deck.klass = RoadClass::Freeway;
    deck.yAbs = { 9.0, 9.0 };
    UnionSpine street;                             // no yAbs -> draped at grade
    street.points = { Vec2(-40, 0), Vec2(40, 0) };
    street.halfWidth = 5.0;
    WeldSolidParams p;
    p.thickness = 0.5; p.clearance = 5.0;
    p.heightAt = [](double, double) { return 0.0; };
    return weldSolid({ deck, street }, p);
}
}  // namespace

// N — two surfaces over the crossing, clear air between (not one merged sheet).
TEST_CASE(gradesep_two_surfaces_over_the_crossing) {
    RenderMesh m = flyover();
    CHECK(!hasNonFinite(m));
    CHECK(indicesInRange(m));
    CHECK(degenerateTriangles(m) == 0);
    CHECK(upwardFraction(m) > 0.3);
    int low = 0, high = 0, mid = 0;
    for (double h : surfaceHitsAt(m, 0.0, 0.0)) {
        if (h < 0.6) ++low; else if (h > 8.0) ++high; else ++mid;
    }
    CHECK(low >= 1);                                // the street at grade
    CHECK(high >= 1);                               // the deck aloft
    CHECK(mid == 0);                                // nothing merged in between
    CHECK(hasClearSpanAt(m, 0.0, 0.0, 0.6, 8.0));   // clear air / sky under the deck
}

// O — no curtain wall: no geometry in the mid-height band over the crossing.
TEST_CASE(gradesep_no_curtain_wall) {
    RenderMesh m = flyover();
    CHECK(verticesInBox(m, -5.0, 5.0, 1.0, 8.0, -5.0, 5.0) == 0);
}

// P — the street is continuous UNDER the deck and the deck continuous OVER it.
TEST_CASE(gradesep_continuous_under_and_over) {
    RenderMesh m = flyover();
    for (double x = -30; x <= 30; x += 5.0) {       // march the street under the deck
        bool grade = false;
        for (double h : surfaceHitsAt(m, x, 0.0)) if (h < 1.0) grade = true;
        CHECK(grade);
    }
    for (double z = -30; z <= 30; z += 5.0) {       // march over the deck
        bool deck = false;
        for (double h : surfaceHitsAt(m, 0.0, z)) if (h > 8.0) deck = true;
        CHECK(deck);
    }
}

// Q — piers straddle the street: columns on both approaches, none on the road.
TEST_CASE(gradesep_piers_straddle_the_street) {
    RenderMesh m = flyover();
    int onRoad = 0, north = 0, south = 0;
    for (const Vertex& v : m.vertices) {
        if (std::fabs(v.position.y) > 0.02 || std::fabs(v.position.x) > 1.2) continue;
        const double z = v.position.z;
        if (std::fabs(z) < 6.0) ++onRoad; else if (z < -6.0) ++north; else if (z > 6.0) ++south;
    }
    CHECK(onRoad == 0);
    CHECK(north > 0);
    CHECK(south > 0);
}

// R — end-to-end through buildRoadNetMesh: an authored elevated E-W road and a
// SEPARATE at-grade N-S street (no shared node) grade-separate at their crossing.
TEST_CASE(gradesep_end_to_end_via_road_net) {
    const double NaN = std::numeric_limits<double>::quiet_NaN();
    RoadNet n;
    n.nodes = { Vec2(-40, 0), Vec2(0, 0), Vec2(40, 0),   // E-W: 0,1,2 (deck at +9)
                Vec2(0, -40), Vec2(0, 40) };             // N-S: 3,4 (at grade)
    n.edges = { {0, 1}, {1, 2}, {3, 4} };
    n.nodeElev = { 9.0, 9.0, 9.0, NaN, NaN };
    n.heightAt = [](double, double) { return 0.0; };
    RenderMesh m = buildRoadNetMesh(n);
    CHECK(!hasNonFinite(m));
    CHECK(degenerateTriangles(m) == 0);
    bool grade = false, deck = false;
    for (double h : surfaceHitsAt(m, 0.0, 0.0)) { if (h < 1.0) grade = true; if (h > 8.0) deck = true; }
    CHECK(grade);                                   // the street passes under
    CHECK(deck);                                    // the deck flies over
    CHECK(verticesInBox(m, -4.0, 4.0, 1.0, 8.0, -4.0, 4.0) == 0);   // no curtain wall
}

// S — ramp foot welds while the crest grade-separates. An E-W road climbs 0->9,
// stays elevated, descends; a cross street meets its WEST foot at grade, and a
// street passes UNDER the elevated crest. The split reclassifies the low ramp
// feet to ground (welding the cross street) while the crest stays a viaduct that
// grade-separates from the flown-over street.
TEST_CASE(gradesep_ramp_foot_welds_crest_separates) {
    const double NaN = std::numeric_limits<double>::quiet_NaN();
    RoadNet n;
    n.nodes = {
        Vec2(-150, 0), Vec2(-80, 0), Vec2(80, 0), Vec2(150, 0),   // 0..3: E-W 0->9->9->0
        Vec2(-150, -40), Vec2(-150, 40),                          // 4,5: cross street at west foot
        Vec2(0, -50), Vec2(0, 50) };                              // 6,7: street under the crest
    n.edges = { {0,1}, {1,2}, {2,3}, {4,0}, {0,5}, {6,7} };
    n.nodeElev = { 0.0, 9.0, 9.0, 0.0, NaN, NaN, NaN, NaN };
    n.heightAt = [](double, double) { return 0.0; };
    RenderMesh m = buildRoadNetMesh(n);
    CHECK(!hasNonFinite(m));
    CHECK(degenerateTriangles(m) == 0);
    // Crest over the flown-over street (x=0): deck aloft + street at grade, clear between.
    bool grade = false, deck = false;
    for (double h : surfaceHitsAt(m, 0.0, 0.0)) { if (h < 1.0) grade = true; if (h > 8.0) deck = true; }
    CHECK(grade);
    CHECK(deck);
    CHECK(verticesInBox(m, -4.0, 4.0, 1.0, 8.0, -4.0, 4.0) == 0);
    // West foot (x=-150): at grade, one welded surface with the cross street (no
    // second surface aloft, no curtain wall there).
    bool foot = false;
    for (double h : surfaceHitsAt(m, -150.0, 0.0)) if (h < 1.5) foot = true;
    CHECK(foot);
    CHECK(hasClearSpanAt(m, -150.0, 0.0, 2.0, 100.0));   // nothing elevated over the foot
}
