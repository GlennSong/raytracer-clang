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

// J — weldSolid MESHES the elevated deck as a SLAB: the same solid welder that
// builds streets seats the deck at the authored height as a fixed-thickness slab
// riding aloft, NOT a curtain wall skirted down. (No terrain here, so no piers —
// this isolates the deck; fixture M covers the piers that hold it up.)
TEST_CASE(weld_solid_meshes_elevated_deck) {
    UnionSpine s;
    s.points = { Vec2(-40, 0), Vec2(0, 0), Vec2(40, 0) };
    s.halfWidth = 6.0;
    s.klass = RoadClass::Freeway;
    s.yAbs = { 9.0, 9.0, 9.0 };
    WeldSolidParams p;
    p.thickness = 0.5;                             // no heightAt -> no piers/drape
    RenderMesh m = weldSolid({ s }, p);
    CHECK(triangleCount(m) > 0);
    CHECK(!hasNonFinite(m));
    CHECK(indicesInRange(m));
    CHECK(degenerateTriangles(m) == 0);
    CHECK(upwardFraction(m) > 0.3);
    double minY, maxY;
    bboxY(m, minY, maxY);
    // Deck top ~+9; underside a fixed slab a hair below (~8.5). A thin slab aloft.
    CHECK(maxY > 8.0 && maxY < 10.0);
    CHECK(minY > 8.0);
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
