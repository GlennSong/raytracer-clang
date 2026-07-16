#include "test_framework.h"
#include "mesh_invariants.h"

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
    CHECK(upwardFraction(m) > 0.4);                // a real up-facing deck (not flipped)
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
