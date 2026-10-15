// THE COMPLEX (cells.h, issue #105 step 5): cells on one frame sharing faces; the bundle of tubes as the first
// preset, every tube with a way in, the shared walls blank to the lower neighbour's roof.
#include "test_framework.h"

#include "../src/engine/procgen/city/cells.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/space_graph.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {
BuildingParams towerParams(int floors) {
    BuildingParams p;
    p.floors = floors;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 77;
    p.faceDir = Vec3(0, 0, 1);   // the street to +z
    return p;
}
int partyIndexFacing(const BuildingParams& p, const Vec2& n) {
    for (int k = 0; k < p.partyWalls; ++k)
        if (dot(p.partyN[k], n) > 0.999) return k;
    return -1;
}
}  // namespace

TEST_CASE(a_bundle_of_nine_tubes_shares_its_faces_and_enters_every_tube) {
    const Poly2 box = {{-30, -22.5}, {30, -22.5}, {30, 22.5}, {-30, 22.5}};
    const BuildingParams p = towerParams(40);
    // rows from -z (the back) to +z (the street): the centre tube the tallest, the corners low
    const std::vector<int> floors = {20, 26, 20, 26, 40, 32, 20, 32, 20};
    const Complex cx = bundledTubes(box, p, 3, 3, floors);
    CHECK(cx.cells.size() == 9);
    CHECK(cx.adj.size() == 12);   // six shared faces across, six along
    for (std::size_t i = 0; i < 9; ++i) {
        const CellFrame& c = cx.cells[i];
        const int col = static_cast<int>(i % 3), row = static_cast<int>(i / 3);
        const int neighbours = (col == 1 ? 2 : 1) + (row == 1 ? 2 : 1);
        CHECK(c.params.partyWalls == neighbours);
        CHECK(c.params.floors == floors[i]);
        CHECK(c.outerFace == (col != 1 || row != 1));
        CHECK(c.connected);
        CHECK(std::fabs(area(c.plan) - 20.0 * 15.0) < 1e-6);
    }
    // The centre tube (40 floors) over its four neighbours: each party wall stops at that neighbour's top.
    const CellFrame& mid = cx.cells[4];
    CHECK(mid.params.partyTop[partyIndexFacing(mid.params, Vec2(0, -1))] == 26);   // the back neighbour (26 floors)
    CHECK(mid.params.partyTop[partyIndexFacing(mid.params, Vec2(0, 1))] == 32);    // the street-side neighbour
    CHECK(mid.params.partyTop[partyIndexFacing(mid.params, Vec2(-1, 0))] == 26);
    CHECK(mid.params.partyTop[partyIndexFacing(mid.params, Vec2(1, 0))] == 32);
    // ...and a low corner's walls run to its own roof (0: blank the whole way; the taller neighbour has the windows).
    const CellFrame& corner = cx.cells[0];
    for (int k = 0; k < corner.params.partyWalls; ++k) CHECK(corner.params.partyTop[k] == 0);
    // The centre tube has no outer face: its entrance faces the street-side neighbour, whose wall takes the doorway.
    CHECK(mid.params.faceDir.z > 0.99 && std::fabs(mid.params.faceDir.x) < 1e-6);
    const CellFrame& front = cx.cells[7];
    const int kDoor = partyIndexFacing(front.params, Vec2(0, -1));
    CHECK(kDoor >= 0 && ((front.params.partyDoors >> kDoor) & 1u) == 1u);
    CHECK(mid.params.partyDoors == 0);   // its own way in is its entrance
    int openings = 0;
    for (const Adjacency& ad : cx.adj) openings += static_cast<int>(ad.openings.size());
    CHECK(openings == 1);
    // Outer tubes enter from an outer face: the street row faces +z, the back row -z, the sides out.
    CHECK(cx.cells[6].params.faceDir.z > 0.99 && cx.cells[8].params.faceDir.z > 0.99);
    CHECK(cx.cells[0].params.faceDir.z < -0.99 || std::fabs(cx.cells[0].params.faceDir.x) > 0.99);
    CHECK(std::fabs(cx.cells[3].params.faceDir.x) > 0.99);
    // Every tube is a building that walks: the gate on each, the inner one through its party-wall entrance.
    for (std::size_t i = 0; i < 9; ++i) {
        const CellFrame& c = cx.cells[i];
        const SpaceGraph g = spaceGraphOf(c.plan, c.params, 0.0);
        int street = 0;
        for (const Portal& po : g.portals) street += po.b < 0;
        CHECK(street >= 1);
        CHECK(unreachableCells(g).empty());
    }
    // The front tube's party wall toward the centre cuts the doorway: that edge is a party door edge, the others not.
    int doorEdges = 0;
    for (std::size_t e = 0; e < front.plan.size(); ++e) doorEdges += partyDoorEdge(front.plan, front.params, e);
    CHECK(doorEdges == 1);
    std::printf("    [bundle] 9 tubes, %zu shared faces, %d opening(s); every tube connected and walkable\n", cx.adj.size(), openings);
}

TEST_CASE(party_walls_stop_at_the_lower_neighbours_roof) {
    const Poly2 box = {{0, 0}, {40, 0}, {40, 20}, {0, 20}};
    BuildingParams p = towerParams(30);
    const Complex cx = bundledTubes(box, p, 2, 1, {30, 12});
    CHECK(cx.cells.size() == 2 && cx.adj.size() == 1);
    const CellFrame& tall = cx.cells[0];
    const CellFrame& low = cx.cells[1];
    CHECK(tall.params.partyWalls == 1 && tall.params.partyTop[0] == 12);
    CHECK(low.params.partyWalls == 1 && low.params.partyTop[0] == 0);
    // The tall tube's shared edge: a party wall on storeys 1..12, windows above.
    std::size_t shared = tall.plan.size();
    for (std::size_t e = 0; e < tall.plan.size(); ++e)
        if (partyEdge(tall.plan, tall.params, e)) shared = e;
    CHECK(shared < tall.plan.size());
    CHECK(partyEdge(tall.plan, tall.params, shared, 1));
    CHECK(partyEdge(tall.plan, tall.params, shared, 12));
    CHECK(!partyEdge(tall.plan, tall.params, shared, 13));
    CHECK(!partyEdge(tall.plan, tall.params, shared, 30));
    // The low tube's: blank to its own roof.
    for (std::size_t e = 0; e < low.plan.size(); ++e)
        if (partyEdge(low.plan, low.params, e)) CHECK(partyEdge(low.plan, low.params, e, 12));
    // The storey view agrees: the tall tube's storey 20 has windows on the shared edge (an opening), storey 5 none.
    const std::vector<StoreyPlan> storeys = storeyPlans(tall.plan, tall.params);
    const std::size_t ee = entranceEdgeFor(tall.plan, tall.params);
    int at5 = 0, at20 = 0;
    for (const WallOpening& o : storeyOpenings(storeys[5].plan, tall.params, 5, storeys[5].y0, storeys[5].h, ee)) {
        const Vec2 m = (o.a + o.b) * 0.5;
        if (std::fabs(m.x - 20.0) < 0.6) ++at5;
    }
    for (const WallOpening& o : storeyOpenings(storeys[20].plan, tall.params, 20, storeys[20].y0, storeys[20].h, ee)) {
        const Vec2 m = (o.a + o.b) * 0.5;
        if (std::fabs(m.x - 20.0) < 0.6) ++at20;
    }
    CHECK(at5 == 0);
    CHECK(at20 > 0);
    std::printf("    [party] shared edge %zu: openings at storey 5: %d, at storey 20: %d\n", shared, at5, at20);
}

TEST_CASE(the_complex_is_a_mass_of_its_cells) {
    const Poly2 box = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    BuildingParams p = towerParams(24);
    const Complex cx = bundledTubes(box, p, 2, 2, {24, 10, 10, 10});
    const Mass& m = cx;
    CHECK(std::fabs(m.height() - (p.groundHeight + 24 * p.floorHeight)) < 1e-9);
    CHECK(m.slice(1.0).pieces.size() == 4);                                        // the ground: every tube
    CHECK(m.slice(p.groundHeight + 15 * p.floorHeight).pieces.size() == 1);        // storey 16: the tall one only
    const MassBounds b = m.bounds();
    CHECK(b.lo.x >= -1e-9 && b.hi.x <= 40 + 1e-9 && b.y1 == m.height());
    const RenderMesh shell = m.shell(0, Vec3(0.5, 0.5, 0.5));
    CHECK(!shell.vertices.empty() && shell.indices.size() % 3 == 0);
}
