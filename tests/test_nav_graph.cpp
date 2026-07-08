#include "test_framework.h"

#include "../src/engine/ai/nav_graph.h"

#include <cmath>

using namespace engine;

namespace {

// A single straight road A(0,0)-B(10,0), width 8, Local.
RoadGraph straightRoad() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(10, 0)} };
    g.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    return g;
}

}  // namespace

TEST_CASE(nav_two_way_doubles_edges) {
    NavGraph nav = buildNavGraph(straightRoad());
    CHECK(nav.nodeCount() == 2);
    // A two-way road yields one link per direction.
    CHECK(nav.linkCount() == 2);
    CHECK(nav.outLinks[0].size() == 1);
    CHECK(nav.outLinks[1].size() == 1);
}

TEST_CASE(nav_ramp_is_one_way) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(10, 0)} };
    g.edges = { RoadEdge{0, 1, 8, RoadClass::Ramp, 0} };
    NavGraph nav = buildNavGraph(g);
    CHECK(nav.linkCount() == 1);
    CHECK(nav.outLinks[0].size() == 1);
    CHECK(nav.outLinks[1].empty());
}

TEST_CASE(nav_link_geometry) {
    NavGraph nav = buildNavGraph(straightRoad());
    int fwd = nav.outLinks[0][0];
    CHECK_APPROX(nav.links[fwd].length, 10.0, 1e-9);
    Vec2 d = nav.direction(fwd);
    CHECK_APPROX(d.x, 1.0, 1e-9);
    CHECK_APPROX(d.y, 0.0, 1e-9);
    Vec2 mid = nav.pointOnLink(fwd, 0.5);
    CHECK_APPROX(mid.x, 5.0, 1e-9);
    CHECK_APPROX(mid.y, 0.0, 1e-9);
}

TEST_CASE(nav_lane_count_from_width) {
    // Local 8m -> 1 lane each way; arterial 16m -> 2 lanes each way.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(10, 0)} };
    g.edges = { RoadEdge{0, 1, 16, RoadClass::Arterial, 0} };
    NavGraph nav = buildNavGraph(g);
    CHECK(nav.links[0].lanes == 2);

    NavGraph local = buildNavGraph(straightRoad());
    CHECK(local.links[0].lanes == 1);
}

TEST_CASE(nav_opposing_lanes_separate_sides) {
    NavGraph nav = buildNavGraph(straightRoad());
    int fwd = nav.outLinks[0][0];   // A->B
    int rev = nav.outLinks[1][0];   // B->A
    Vec2 lf = nav.laneCenter(fwd, 0, 0.5, 3.5);
    Vec2 lr = nav.laneCenter(rev, 0, 0.5, 3.5);
    // Each lane sits half a lane width off the centreline...
    CHECK_APPROX(std::fabs(lf.y), 1.75, 1e-9);
    CHECK_APPROX(std::fabs(lr.y), 1.75, 1e-9);
    // ...on OPPOSITE sides, so oncoming traffic never overlaps.
    CHECK(lf.y * lr.y < 0);
    // Their midpoint is back on the centreline.
    CHECK_APPROX((lf.y + lr.y) * 0.5, 0.0, 1e-9);
}

TEST_CASE(nav_sidewalk_outside_carriageway) {
    NavGraph nav = buildNavGraph(straightRoad());
    int fwd = nav.outLinks[0][0];
    Vec2 s = nav.sidewalkPoint(fwd, 0.5, 1.0);
    // width 8 -> half-width 4, +1 verge = 4.998..; outside the lane (1.75).
    CHECK_APPROX(std::fabs(s.y), 5.0, 1e-9);
}

TEST_CASE(nav_flags_junctions_by_degree) {
    // A plus/cross: centre node 0 with four arms (1..4). The centre is a junction
    // (4 neighbours); the arm tips are not (1 neighbour each).
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(10, 0)}, {Vec2(-10, 0)},
                {Vec2(0, 10)}, {Vec2(0, -10)} };
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0},
        RoadEdge{0, 2, 8, RoadClass::Local, 0},
        RoadEdge{0, 3, 8, RoadClass::Local, 0},
        RoadEdge{0, 4, 8, RoadClass::Local, 0},
    };
    NavGraph nav = buildNavGraph(g);
    CHECK(nav.isJunction(0));
    CHECK(!nav.isJunction(1));
    CHECK(!nav.isJunction(4));
}

TEST_CASE(nav_straight_road_has_no_junction) {
    NavGraph nav = buildNavGraph(straightRoad());
    CHECK(!nav.isJunction(0));
    CHECK(!nav.isJunction(1));
}

TEST_CASE(nav_nearest_queries) {
    NavGraph nav = buildNavGraph(straightRoad());
    CHECK(nav.nearestNode(Vec2(0.4, 0.1)) == 0);
    CHECK(nav.nearestNode(Vec2(9.7, -0.2)) == 1);
    // A point beside the middle of the road is nearest some link, not off-graph.
    int li = nav.nearestLink(Vec2(5, 2));
    CHECK(li >= 0);
}

TEST_CASE(nav_junction_knot_merges_into_one_intersection) {
    // Two 4-way junction nodes 5 m apart, joined by a car-length stub (the shape
    // a curvy generated crossing planarizes into). buildNavGraph must merge them
    // into ONE intersection (ADR-0066 gridlock fix), record how far the members
    // sat from the centroid (nodeSpread — signal poles back off by it), and keep
    // far-apart degree-2 nodes untouched.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)},   {Vec2(5, 0)},              // the knot pair
                {Vec2(-60, 0)}, {Vec2(65, 0)},             // long approaches
                {Vec2(0, 60)},  {Vec2(0, -60)},
                {Vec2(5, 60)},  {Vec2(5, -60)} };
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0},            // the stub
        RoadEdge{2, 0, 8, RoadClass::Local, 0},
        RoadEdge{1, 3, 8, RoadClass::Local, 0},
        RoadEdge{4, 0, 8, RoadClass::Local, 0},
        RoadEdge{0, 5, 8, RoadClass::Local, 0},
        RoadEdge{6, 1, 8, RoadClass::Local, 0},
        RoadEdge{1, 7, 8, RoadClass::Local, 0},
    };
    NavGraph nav = buildNavGraph(g);
    CHECK(nav.nodeCount() == 7);                 // 8 nodes, knot pair merged to 1
    // Exactly one junction, at the knot centroid, carrying the members' spread.
    int junctions = 0, merged = -1;
    for (int i = 0; i < nav.nodeCount(); ++i)
        if (nav.isJunction(i)) { ++junctions; merged = i; }
    CHECK(junctions == 1);
    CHECK_APPROX(nav.nodes[merged].x, 2.5, 1e-9);
    CHECK_APPROX(nav.nodes[merged].y, 0.0, 1e-9);
    CHECK_APPROX(nav.nodeSpread[merged], 2.5, 1e-9);   // members sat 2.5 m out
    // Every surviving link still routes (no stub self-loop, no orphan endpoints).
    for (const NavLink& l : nav.links) {
        CHECK(l.from != l.to);
        CHECK(l.length > 1.0);
    }
    // Ordinary graphs are untouched: no merge, zero spread everywhere.
    NavGraph plain = buildNavGraph(straightRoad());
    for (Real s : plain.nodeSpread) CHECK(s == 0.0);
}
