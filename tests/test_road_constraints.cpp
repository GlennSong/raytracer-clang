#include "test_framework.h"

#include "../src/engine/procgen/city/road_constraints.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include <cmath>

using namespace engine;

namespace {

// A hub: one centre node with `spokes` arms radiating out evenly to radius `len`.
RoadGraph radialHub(int spokes, double len = 60.0, double width = 12.0) {
    RoadGraph g;
    g.nodes.push_back({Vec2(0, 0)});                 // node 0 = the hub
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        int n = static_cast<int>(g.nodes.size());
        g.nodes.push_back({Vec2(std::cos(a) * len, std::sin(a) * len)});
        g.addEdge(0, n, width, RoadClass::Local);
    }
    return g;
}

int degreeOf(const RoadGraph& g, int v) {
    int d = 0;
    for (const RoadEdge& e : g.edges) if (e.a == v || e.b == v) ++d;
    return d;
}

int maxDegree(const RoadGraph& g) {
    int mx = 0;
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) mx = std::max(mx, degreeOf(g, v));
    return mx;
}

bool everyNodeReferenced(const RoadGraph& g) {
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v)
        if (degreeOf(g, v) == 0) return false;
    return true;
}

}  // namespace

// A clean 4-way crossing is a flat patch, not a roundabout: nothing should change.
TEST_CASE(constraints_leave_a_four_way_alone) {
    RoadGraph g = radialHub(4);
    CHECK(!nodeNeedsRoundabout(g, 0, {}));
    RoadGraph out = applyConstraints(g);
    CHECK(out.nodes.size() == g.nodes.size());
    CHECK(out.edges.size() == g.edges.size());
    CHECK(maxDegree(out) == 4);
}

// A 3-way T with healthy angles also stays flat.
TEST_CASE(constraints_leave_a_tee_alone) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(-30, 0)}, {Vec2(30, 0)}, {Vec2(0, 30)} };
    g.addEdge(0, 1); g.addEdge(0, 2); g.addEdge(0, 3);
    CHECK(!nodeNeedsRoundabout(g, 0, {}));
    CHECK(applyConstraints(g).edges.size() == g.edges.size());
}

// Many spokes -> the hub promotes to a roundabout: the degree-N super-node is gone and
// every surviving node is degree <= 3 (the case the analytic junction pad handles).
TEST_CASE(constraints_promote_a_busy_hub) {
    RoadGraph g = radialHub(8);
    CHECK(nodeNeedsRoundabout(g, 0, {}));
    RoadGraph out = applyConstraints(g);
    CHECK(maxDegree(out) <= 3);
    CHECK(out.edges.size() > g.edges.size());     // ring arcs were added
    CHECK(everyNodeReferenced(out));              // the super-node was compacted away
    // All 8 spokes survive: 8 nodes still sit at the original spoke radius (~60 m).
    int rim = 0;
    for (const RoadNode& n : out.nodes)
        if (std::fabs(n.pos.length() - 60.0) < 1e-3) ++rim;
    CHECK(rim == 8);
}

// Two arms too acute to share a flat junction promote even below the degree cap.
TEST_CASE(constraints_promote_an_acute_fork) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(60, 0)}, {Vec2(60, 6)}, {Vec2(-40, 30)} };
    g.addEdge(0, 1); g.addEdge(0, 2); g.addEdge(0, 3);   // arms 1 and 2 ~6 deg apart
    CHECK(degreeOf(g, 0) == 3);
    CHECK(degreeOf(g, 0) <= RoadRules{}.maxDegree);       // not promoted for being busy
    CHECK(nodeNeedsRoundabout(g, 0, {}));                 // promoted for being acute
    CHECK(maxDegree(applyConstraints(g)) <= 3);
}

// The auto radius answers the spokes: more arms (tighter packing) -> a bigger ring.
TEST_CASE(constraints_radius_grows_with_spokes) {
    auto ringExtent = [](const RoadGraph& g) {
        RoadGraph out = applyConstraints(g);
        double mx = 0;                              // largest non-spoke radius ~ the ring radius
        for (const RoadNode& n : out.nodes) {
            double r = n.pos.length();
            if (r < 55.0) mx = std::max(mx, r);
        }
        return mx;
    };
    double r8 = ringExtent(radialHub(8));
    double r16 = ringExtent(radialHub(16));
    CHECK(r16 > r8);
}


// With auto-roundabouts OFF (the city generator's policy), a busy hub is left untouched — no ring
// is added (capDegree handles the degree instead), and nodeNeedsRoundabout reports false.
TEST_CASE(constraints_skip_auto_roundabout_when_disabled) {
    RoadGraph g = radialHub(8);
    RoadRules rules; rules.autoRoundabout = false;
    CHECK(!nodeNeedsRoundabout(g, 0, rules));
    RoadGraph out = applyConstraints(g, rules);
    CHECK(out.nodes.size() == g.nodes.size());     // unchanged: no ring
    CHECK(out.edges.size() == g.edges.size());
}

// Field-carry (semantic-layer S0): the promotion rebuild used to re-init
// edges positionally, silently resetting every field after `layer`
// (walkable, spec, oneWay, provenance — and the access bits the semantic
// layer adds). A promoted hub must keep its surviving spokes' payload.
TEST_CASE(constraints_promotion_preserves_edge_payload) {
    RoadGraph g = radialHub(7);   // over maxDegree: promotes to a roundabout
    for (RoadEdge& e : g.edges) {
        e.walkable = false;       // non-default on every spoke
        e.spec = 3;
        e.oneWay = true;
        e.provenance = RoadProvenance::CorridorRamp;
    }
    RoadGraph out = applyConstraints(g);
    CHECK(out.nodes.size() > g.nodes.size());   // it really promoted
    // Ring arcs are synthesized (default payload, and they inherit the
    // spokes' width — width cannot tell them apart). The payload itself
    // identifies the surviving spokes: exactly the original 7 edges carry
    // it, and each carries ALL of it.
    int spokes = 0;
    for (const RoadEdge& e : out.edges) {
        if (e.spec != 3) continue;      // a synthesized ring arc
        ++spokes;
        CHECK(!e.walkable);
        CHECK(e.oneWay);
        CHECK(e.provenance == RoadProvenance::CorridorRamp);
    }
    CHECK(spokes == 7);   // every spoke survived with its payload
}
