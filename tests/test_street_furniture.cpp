// Street furniture placement (build-time poles, sim-time phases). The claims
// under test are the ones the device drives exposed:
//   - "double stoplights which is bizarre": overlapping collinear approaches
//     must collapse to ONE pole per (junction, bearing).
//   - poles agree with the CONTROLLER: no pole where SignalController leaves
//     the junction uncontrolled (T-junctions, by device feedback).
#include "test_framework.h"

#include "../src/engine/procgen/city/street_furniture.h"

using namespace engine;

namespace {

// A 4-way cross at the origin with a COLLINEAR DUPLICATE approach: the east
// arm exists twice — the full edge and a stub whose far node rides ON the
// full edge's segment (the exact geometry measured on metropolis: node 167's
// arm entered by both a 74.6 m edge and a 4.9 m collinear stub).
NavGraph crossWithCollinearStub() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)},            // 0: the junction
                {Vec2(0, 60)},  {Vec2(0, -60)},   // 1,2: north/south tips
                {Vec2(60, 0)},  {Vec2(-60, 0)},   // 3,4: east/west tips
                {Vec2(8, 0)} };                    // 5: ON the east arm
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0},
        RoadEdge{0, 2, 8, RoadClass::Local, 0},
        RoadEdge{0, 3, 8, RoadClass::Local, 0},   // east, full
        RoadEdge{0, 4, 8, RoadClass::Local, 0},
        RoadEdge{0, 5, 8, RoadClass::Local, 0},   // east, collinear stub
        RoadEdge{5, 3, 8, RoadClass::Local, 0},   // stub continues to the tip
    };
    NavBuildParams np;
    np.junctionMergeRadius = 0;   // keep the raw duplicate (the defect's shape)
    return buildNavGraph(g, np);
}

NavGraph tee() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(0, 60)}, {Vec2(60, 0)}, {Vec2(-60, 0)} };
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0},
        RoadEdge{0, 2, 8, RoadClass::Local, 0},
        RoadEdge{0, 3, 8, RoadClass::Local, 0},
    };
    return buildNavGraph(g);
}

}  // namespace

TEST_CASE(signal_poles_dedupe_collinear_approaches) {
    NavGraph nav = crossWithCollinearStub();
    StreetFurniturePlan plan =
        planStreetFurniture(nav, [](Real, Real) { return Real(0); }, {});
    // Node 0 is entered by FIVE street links (N, S, W, east-full, east-stub);
    // the two east approaches share a bearing, so exactly FOUR poles stand.
    int atCentre = 0;
    for (const SignalSpot& s : plan.signals) {
        const NavLink& L = nav.links[s.link];
        if ((nav.nodes[L.to] - Vec2(0, 0)).length() < 1.0) ++atCentre;
    }
    CHECK(atCentre == 4);
}

TEST_CASE(no_signal_poles_at_uncontrolled_tees) {
    // SignalController leaves 3-approach junctions uncontrolled (device: "a
    // dense district isn't a forest of stoplights") — so no pole may stand
    // there either. A dark, dead head on every T is what the per-link
    // placement used to build.
    NavGraph nav = tee();
    StreetFurniturePlan plan =
        planStreetFurniture(nav, [](Real, Real) { return Real(0); }, {});
    CHECK(plan.signals.empty());
}
