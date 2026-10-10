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

// A 4-way whose east arm has a neighbour leaving at only 30 degrees — the
// metro's organic grid is full of these. Approaching from the east, the pole's
// kerb-corner sits between the east arm and that acute neighbour.
NavGraph acuteCross() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)},
                {Vec2(0, 60)}, {Vec2(0, -60)}, {Vec2(60, 0)},
                {Vec2(52, 30)} };                   // 30 degrees off the east arm
    g.edges = {
        RoadEdge{0, 1, 12, RoadClass::Local, 0},
        RoadEdge{0, 2, 12, RoadClass::Local, 0},
        RoadEdge{0, 3, 12, RoadClass::Local, 0},
        RoadEdge{0, 4, 12, RoadClass::Local, 0},
    };
    return buildNavGraph(g);
}

}  // namespace

TEST_CASE(signal_poles_clear_every_arm_at_an_acute_corner) {
    // Metro device report: "stoplights show up in the middle of the street and
    // not on the corners" — every offender stood between two arms meeting at
    // an acute angle, inside the NEIGHBOUR's carriageway. A pole must clear
    // every arm at its node by (half-width + curbGap), not just its own.
    NavGraph nav = acuteCross();
    StreetFurnitureParams fp;
    StreetFurniturePlan plan =
        planStreetFurniture(nav, [](Real, Real) { return Real(0); }, fp);
    CHECK(plan.signals.size() == 4);
    int acuteChecked = 0;
    for (const SignalSpot& s : plan.signals) {
        const NavLink& L = nav.links[s.link];
        const Vec2 node = nav.nodes[L.to];
        const Vec2 base(s.base.x, s.base.z);
        for (int li = 0; li < nav.linkCount(); ++li) {
            const NavLink& K = nav.links[li];
            if (K.from != L.to) continue;              // arms leave the node
            const Vec2 e = nav.direction(li);
            const Vec2 q = base - node;
            const Real along = dot(q, e);
            if (along <= 0) continue;                  // behind the node
            const Real perp = std::fabs(e.x * q.y - e.y * q.x);
            CHECK(perp >= K.width * 0.5 + fp.curbGap - 1e-6);
            if (li != s.link) ++acuteChecked;
        }
        // Still kerb-adjacent on its own arm.
        const Vec2 d = nav.direction(s.link);
        const Vec2 right(d.y, -d.x);
        const Real lateral = dot(base - node, right);
        CHECK(std::fabs(lateral - (L.width * 0.5 + fp.curbGap)) < 1e-6);
    }
    CHECK(acuteChecked > 0);   // the fixture must exercise a neighbour
}

TEST_CASE(lamps_never_stand_in_another_streets_carriageway) {
    // Metro map (device): "some of the street lights are actually sitting
    // right in the middle of the road". Lamps cleared junctions by a radius
    // around the NODE; at an acute corner the neighbouring arm's carriageway
    // reaches past it. On this fixture the east arm's right-kerb lamps at
    // 10..24 m from the node sit inside the 30-degree arm's 12 m ribbon.
    NavGraph nav = acuteCross();
    StreetFurnitureParams fp;
    StreetFurniturePlan plan =
        planStreetFurniture(nav, [](Real, Real) { return Real(0); }, fp);
    CHECK(!plan.lampBases.empty());
    int checked = 0;
    for (const Vec3& b : plan.lampBases) {
        const Vec2 v(b.x, b.z);
        for (int li = 0; li < nav.linkCount(); ++li) {
            const NavLink& K = nav.links[li];
            const Vec2 a = nav.nodes[K.from], e = nav.nodes[K.to];
            const Vec2 ab = e - a;
            Real t = dot(v - a, ab) / ab.lengthSquared();
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            const Real d = (a + ab * t - v).length();
            CHECK(d >= K.width * 0.5 + fp.curbGap - 1e-6);
            ++checked;
        }
    }
    CHECK(checked > 0);
}

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

TEST_CASE(signal_poles_stand_clear_of_the_junction_pad) {
    // Glenn: "stoplights are placed out in the middle of the road." The drawn
    // junction pad fills the disc out to (widest half-width + sidewalk), so a
    // kerb pole must back off past THAT along its approach — the old formula
    // used only the carriageway half-width and stood every pole a
    // sidewalk-width deep in the asphalt. The mast arm, not the pole, reaches
    // over the road.
    NavGraph nav = crossWithCollinearStub();
    StreetFurnitureParams fp;
    fp.sidewalkWidth = 3.5;
    StreetFurniturePlan plan =
        planStreetFurniture(nav, [](Real, Real) { return Real(0); }, fp);
    CHECK(!plan.signals.empty());
    for (const SignalSpot& s : plan.signals) {
        const NavLink& L = nav.links[s.link];
        const Vec2 node = nav.nodes[L.to];
        Real crossHalf = L.width * 0.5;
        for (int ol : nav.outLinks[L.to])
            crossHalf = std::max(crossHalf, nav.links[ol].width * 0.5);
        const Real padRadius = crossHalf + fp.sidewalkWidth;
        // The pole's distance from the junction node must clear the pad disc.
        const Vec2 base(s.base.x, s.base.z);
        CHECK((base - node).length() > padRadius);
        // ...while staying kerb-adjacent laterally: within the sidewalk band
        // beside its own carriageway, not wandering into the block.
        const Vec2 d = nav.direction(s.link);
        const Vec2 right(d.y, -d.x);
        const Real lateral = dot(base - node, right);
        CHECK(lateral > L.width * 0.5 - 1e-6);
        CHECK(lateral < L.width * 0.5 + fp.sidewalkWidth + fp.curbGap + 1e-6);
    }
}

// KERB LIFE (the city's next ten #3): a long collector crossing a local street gets street trees on both kerbs at
// their spacing, a litter bin by the corner, and racks or news boxes mid-block -- every piece on the pavement (beyond
// the kerb, clear of every carriageway), the trees clear of the lamps and the junction's sightlines.
TEST_CASE(kerb_life_lines_the_pavements_and_keeps_off_the_road) {
    RoadGraph g;
    const int w = g.addNode({-300, 0}), e = g.addNode({300, 0}), n = g.addNode({0, 300}), s = g.addNode({0, -300}), c = g.addNode({0, 0});
    g.addEdge(w, c, 12, RoadClass::Collector); g.addEdge(c, e, 12, RoadClass::Collector);
    g.addEdge(s, c, 10, RoadClass::Local); g.addEdge(c, n, 10, RoadClass::Local);
    NavGraph nav = buildNavGraph(g);
    for (NavLink& L : nav.links) L.access |= road_access::kFrontage;
    StreetFurnitureParams fp;
    const StreetFurniturePlan plan = planStreetFurniture(nav, [](Real, Real) { return Real(0); }, fp);
    int bins = 0, mid = 0;
    for (const KerbProp& kp : plan.props) (kp.kind == KerbProp::Bin ? bins : mid)++;
    std::printf("    [kerb] %zu trees, %d bins, %d racks/news boxes, %zu lamps\n", plan.trees.size(), bins, mid, plan.lampBases.size());
    // ~1100 m of street x 2 kerbs at 9 m, less the junction's clear zone and the lamps' room
    CHECK(plan.trees.size() > 150);
    CHECK(bins >= 4);   // a corner bin on each approach's kerb
    CHECK(mid >= 4);
    auto offRoad = [&](Vec2 v) {
        for (int li = 0; li < nav.linkCount(); ++li) {
            const NavLink& K = nav.links[li];
            const Vec2 a = nav.nodes[K.from], b = nav.nodes[K.to], ab = b - a;
            Real t = dot(v - a, ab) / ab.lengthSquared(); t = std::max(Real(0), std::min(Real(1), t));
            if ((a + ab * t - v).length() < K.width * 0.5 + 0.3) return false;
        }
        return true;
    };
    int onRoad = 0, byLamp = 0, atCorner = 0;
    for (const StreetTree& t : plan.trees) {
        const Vec2 v(t.base.x, t.base.z);
        if (!offRoad(v)) ++onRoad;
        if (v.length() < fp.treeJunctionClear - 0.5) ++atCorner;
        for (const Vec3& l : plan.lampBases) if ((Vec2(l.x, l.z) - v).length() < 3.0) { ++byLamp; break; }
    }
    for (const KerbProp& kp : plan.props) if (!offRoad(Vec2(kp.base.x, kp.base.z))) ++onRoad;
    CHECK(onRoad == 0);
    CHECK(byLamp == 0);
    CHECK(atCorner == 0);
    // a tree on BOTH kerbs of the collector
    int northKerb = 0, southKerb = 0;
    for (const StreetTree& t : plan.trees) if (std::fabs(t.base.x) > 30) (t.base.z > 0 ? northKerb : southKerb)++;
    CHECK(northKerb > 30 && southKerb > 30);
    // off: lamps and signals only
    fp.kerbLife = false;
    const StreetFurniturePlan bare = planStreetFurniture(nav, [](Real, Real) { return Real(0); }, fp);
    CHECK(bare.trees.empty() && bare.props.empty());
    CHECK(bare.lampBases.size() == plan.lampBases.size());
}
