#include "test_framework.h"
#include "mesh_invariants.h"

#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/road_net.h"

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
// streets seats the DECK TOP at the authored height, not on the terrain below.
// This is the "one mesher carries the corridor" proof. (The underside currently
// skirts down to the ground — an at-grade assumption; a fixed slab-on-piers
// underside for authored decks is the next welder-goes-3D increment.)
TEST_CASE(weld_solid_meshes_elevated_deck) {
    UnionSpine s;
    s.points = { Vec2(-40, 0), Vec2(0, 0), Vec2(40, 0) };
    s.halfWidth = 6.0;
    s.klass = RoadClass::Freeway;
    s.yAbs = { 9.0, 9.0, 9.0 };
    WeldSolidParams p;
    p.heightAt = [](double, double) { return -15.0; };   // ground well below the deck
    RenderMesh m = weldSolid({ s }, p);
    CHECK(triangleCount(m) > 0);
    CHECK(!hasNonFinite(m));
    CHECK(indicesInRange(m));
    CHECK(degenerateTriangles(m) == 0);
    CHECK(upwardFraction(m) > 0.3);
    double minY, maxY;
    bboxY(m, minY, maxY);
    // Deck top rode to ~+9. If drape had won, the top would sit at the -15 ground.
    CHECK(maxY > 8.0 && maxY < 10.0);
}
