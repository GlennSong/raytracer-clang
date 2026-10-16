// THE COMPLEX (cells.h, issue #105 step 5): cells on one frame sharing faces; the bundle of tubes as the first
// preset, every tube with a way in, the shared walls blank to the lower neighbour's roof.
#include "test_framework.h"

#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/building_collider.h"
#include "../src/engine/procgen/city/cells.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/room_plan.h"
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
    // SHARED LOBBIES (ADR-0179): every whole shared wall opens both ways on the ground, so the centre tube's four
    // walls all carry a doorway (its own way in is still its entrance, on the street-side one), and every one of
    // the twelve shared faces has its ground opening
    CHECK(mid.params.partyWalls == 4 && mid.params.partyDoors == 0xF);
    int openings = 0;
    for (const Adjacency& ad : cx.adj) openings += static_cast<int>(ad.openings.size());
    CHECK(openings == 12);
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
    // The front tube's three party walls (the centre and its two side neighbours) all cut a doorway; its street face
    // is no party wall.
    int doorEdges = 0;
    for (std::size_t e = 0; e < front.plan.size(); ++e) doorEdges += partyDoorEdge(front.plan, front.params, e);
    CHECK(doorEdges == 3);
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

TEST_CASE(a_courtyard_block_is_eight_cells_round_an_open_court) {
    const Poly2 box = {{-30, -22}, {30, -22}, {30, 22}, {-30, 22}};
    BuildingParams p = towerParams(5);
    p.curtainWall = false;
    const Complex cx = courtyardWings(box, p, 15.0, {5, 5, 5, 5, 6, 5, 6, 5});
    CHECK(cx.cells.size() == 8);
    CHECK(cx.adj.size() == 8);   // every corner shares a whole edge with each of its two wings
    Real built = 0;
    for (std::size_t i = 0; i < cx.cells.size(); ++i) {
        const CellFrame& c = cx.cells[i];
        CHECK(c.params.partyWalls == 2);
        CHECK(c.outerFace && c.streetFace && c.connected);
        // every entrance on the block's own boundary, never onto the court: the south wing faces -z, the north +z,
        // the east +x, the west -x, the corners the street side nearest `faceDir` (+z)
        const Vec2 want[8] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {0, -1}, {0, 1}, {0, 1}};
        if (i < 4) CHECK(std::fabs(c.params.faceDir.x - want[i].x) < 1e-6 && std::fabs(c.params.faceDir.z - want[i].y) < 1e-6);
        else CHECK(std::fabs(std::fabs(c.params.faceDir.z) - 1.0) < 1e-6 || std::fabs(std::fabs(c.params.faceDir.x) - 1.0) < 1e-6);
        CHECK(c.params.partyDoors == 3);   // every cell has its own street door, and a doorway through both shared walls (ADR-0179)
        built += area(c.plan);
        // a party wall is a WHOLE edge: an edge along a party plane has both its ends on it (an edge meeting the
        // plane end-on touches it at one corner, which is not a wall on it)
        for (std::size_t e = 0; e < c.plan.size(); ++e) {
            const Vec2 a = c.plan[e], b = c.plan[(e + 1) % c.plan.size()];
            const Vec2 d = normalize(b - a);
            for (int k = 0; k < c.params.partyWalls; ++k) {
                const Vec2 N = c.params.partyN[k];
                if (std::fabs(dot(d, N)) > 1e-6) continue;   // not along the plane
                const bool aOn = std::fabs(dot(a, N) - c.params.partyAt[k]) < 1e-6, bOn = std::fabs(dot(b, N) - c.params.partyAt[k]) < 1e-6;
                CHECK(aOn == bOn);
            }
        }
    }
    CHECK(std::fabs(built - (60.0 * 44.0 - 30.0 * 14.0)) < 1e-6);   // the block less its court
    // The taller corners' walls stop at their wings' roofs; the wings' walls run to their own.
    const CellFrame& sw = cx.cells[4];
    CHECK(sw.params.floors == 6);
    for (int k = 0; k < sw.params.partyWalls; ++k) CHECK(sw.params.partyTop[k] == 5);
    const CellFrame& south = cx.cells[0];
    for (int k = 0; k < south.params.partyWalls; ++k) CHECK(south.params.partyTop[k] == 0);
    // One crown for the block, on its tallest cell; every other roof bare (top 9).
    int crowned = 0;
    for (const CellFrame& c : cx.cells) crowned += c.params.top != 9;
    CHECK(crowned == 1 && cx.cells[4].params.top != 9);
    // Every cell walks; a wing's court face has windows (an outer face, no neighbour on it).
    for (const CellFrame& c : cx.cells) CHECK(unreachableCells(spaceGraphOf(c.plan, c.params, 0.0)).empty());
    // ...and every cell GROWN has its entrance (the lot pass collects it from the facade's attach: no door, not
    // enterable, and the lab's corners came out shut)
    for (int retail = 0; retail < 2; ++retail)
        for (std::size_t i = 0; i < cx.cells.size(); ++i) {
            const CellFrame& c = cx.cells[i];
            BuildingParams gp = c.params;
            gp.groundRetail = retail != 0;   // shops on the ground or not, the lobby door stays
            const BuildingMesh bm = growPlanBuilding(c.plan, gp, 0.0);
            int doors = 0;
            for (const AttachPoint& ap : bm.attaches) doors += ap.tag == "entrance";
            std::printf("    [court] cell %zu (%s) face (%.0f, %.0f) shops %d: %d entrance(s)\n", i, c.role.c_str(), gp.faceDir.x, gp.faceDir.z, retail, doors);
            CHECK(doors >= 1);
        }
    int partyEdges = 0;
    for (std::size_t e = 0; e < south.plan.size(); ++e) partyEdges += partyEdge(south.plan, south.params, e);
    CHECK(partyEdges == 2);
    // Too small for a court: nothing.
    CHECK(courtyardWings({{0, 0}, {36, 0}, {36, 30}, {0, 30}}, p, 15.0, {5}).cells.empty());
    std::printf("    [court] 8 cells, %zu shared faces, built %.0f m2 round a %.0f x %.0f court\n", cx.adj.size(), built, 60.0 - 30.0, 44.0 - 30.0);
}

TEST_CASE(a_ring_of_cells_round_a_court_enters_every_segment_from_outside) {
    Poly2 ngon;
    for (int k = 0; k < 12; ++k) {
        const Real a2 = 2.0 * 3.14159265358979323846 * (k + 0.5) / 12;
        ngon.push_back(Vec2(std::cos(a2), std::sin(a2)) * 30.0);
    }
    BuildingParams p = towerParams(4);
    const Complex cx = ringCells(ngon, p, 12.0, {4});
    CHECK(cx.cells.size() == 12);
    CHECK(cx.adj.size() == 12);   // every radial face shared whole
    Real built = 0;
    for (std::size_t i = 0; i < cx.cells.size(); ++i) {
        const CellFrame& c = cx.cells[i];
        CHECK(c.plan.size() == 4);
        CHECK(c.params.partyWalls == 2);
        CHECK(c.outerFace && c.streetFace && c.connected);
        CHECK(c.params.partyDoors == 3);   // a doorway through both radial walls: the ring's ground floor is one lobby (ADR-0179)
        CHECK(c.params.top == 9);   // a ring's roof is one clean deck
        // the entrance faces outward: along the cell's own radial direction, not into the court
        const Vec2 radial = normalize(centroid(c.plan));
        CHECK(dot(Vec2(c.params.faceDir.x, c.params.faceDir.z), radial) > 0.9);
        built += area(c.plan);
        CHECK(unreachableCells(spaceGraphOf(c.plan, c.params, 0.0)).empty());
    }
    const Poly2 inner = offsetPlanPublic(ngon, 12.0);
    CHECK(std::fabs(built - (area(ngon) - area(inner))) < 1e-6);
    // Too deep for its court: nothing.
    CHECK(ringCells(ngon, p, 28.0, {4}).cells.empty());
    std::printf("    [ring] 12 segments, %.0f m2 built round a court of %.0f m2\n", built, area(inner));
}


// OPENINGS BETWEEN CELLS (ADR-0179): shared lobbies through every whole party wall, and twin towers joined by a sky
// bridge -- a link on both faces, the bridge drawn and walked from one, a lounge storey either end.
TEST_CASE(a_bundle_opens_its_lobbies_through_every_shared_wall) {
    const Poly2 box = {{-30, -30}, {30, -30}, {30, 30}, {-30, 30}};
    BuildingParams p = towerParams(20);
    p.faceDir = Vec3(0, 0, -1);
    const Complex cx = bundledTubes(box, p, 2, 2, {20, 16, 12, 20});
    CHECK(cx.cells.size() == 4);
    CHECK(cx.adj.size() == 4);   // 2 x 2: four shared walls
    int doorsBothSides = 0;
    for (const Adjacency& ad : cx.adj) {
        const CellFrame& ca = cx.cells[static_cast<std::size_t>(ad.a)];
        const CellFrame& cb = cx.cells[static_cast<std::size_t>(ad.b)];
        auto hasDoor = [&](const CellFrame& c, const Vec2& nOut, Real at) {
            for (int k = 0; k < c.params.partyWalls; ++k)
                if (dot(c.params.partyN[k], nOut) > 0.999 && std::fabs(c.params.partyAt[k] - at) < 0.3 && ((c.params.partyDoors >> k) & 1u)) return true;
            return false;
        };
        if (hasDoor(ca, ad.n, ad.at) && hasDoor(cb, ad.n * -1.0, -ad.at)) ++doorsBothSides;
        bool ground = false;
        for (const Opening& o : ad.openings) if (o.k0 == 0) ground = true;
        CHECK(ground);
    }
    CHECK(doorsBothSides == 4);
    // every tube still a building walked base to top
    for (const CellFrame& c : cx.cells) CHECK(unreachableCells(spaceGraphOf(c.plan, c.params, 0.0)).empty());
    std::printf("    [lobbies] %zu shared walls, %d with a doorway both sides\n", cx.adj.size(), doorsBothSides);
}

TEST_CASE(twin_towers_are_joined_by_a_sky_bridge_walked_from_a_lounge) {
    const Poly2 box = {{-34, -14}, {34, -14}, {34, 14}, {-34, 14}};
    BuildingParams p = towerParams(24);
    p.faceDir = Vec3(0, 0, -1);
    const Complex cx = twinTowers(box, p, 12.0, {24, 24}, 0, 4.5);
    CHECK(cx.cells.size() == 2);
    CHECK(cx.adj.empty());   // they stand apart
    CHECK(cx.bridges.size() == 1 && cx.bridges[0].placed);
    const Bridge& br = cx.bridges[0];
    CHECK(std::fabs(br.span - 12.0) < 0.05);
    CHECK(br.storey == 16);   // two thirds of the way up
    const CellFrame& a = cx.cells[0];
    const CellFrame& b = cx.cells[1];
    CHECK(a.params.links == 1 && b.params.links == 1);
    CHECK(a.params.linkStorey[0] == 16 && b.params.linkStorey[0] == 16);
    CHECK(std::fabs(a.params.linkLen[0] - 12.0) < 0.05 && b.params.linkLen[0] == 0);
    CHECK(dot(a.params.linkN[0], b.params.linkN[0]) < -0.999);               // the faces face each other
    CHECK(std::fabs(a.params.linkC[0].y - b.params.linkC[0].y) < 1e-6);        // the doors in line across the gap
    CHECK(std::fabs((b.params.linkC[0] - a.params.linkC[0]).length() - 12.0) < 0.05);
    // the doors, from each side: a's with the bridge footprint, b's a door alone
    const std::vector<LinkDoor> da = linkDoors(a.plan, a.params, 16, 0), db = linkDoors(b.plan, b.params, 16, 0);
    CHECK(da.size() == 1 && db.size() == 1);
    CHECK(da[0].bridge.size() == 4 && db[0].bridge.empty());
    CHECK(std::fabs(da[0].w - 4.5) < 1e-6);
    CHECK(linkDoors(a.plan, a.params, 15, 0).empty());
    // the grammar: a's building carries the bridge (its floor and sides for the collider, glass for the eye)
    const BuildingMesh ma = growPlanBuilding(a.plan, a.params, 0.0);
    int bridgeItems = 0, wallItems = 0;
    for (const RoofItem& ri : ma.roof) { if (ri.kind == RoofItem::Kind::Bridge) ++bridgeItems; if (ri.kind == RoofItem::Kind::BridgeWall) ++wallItems; }
    CHECK(bridgeItems == 1 && wallItems == 2);
    BuildingParams plain = a.params;
    plain.links = 0;
    const BuildingMesh mp = growPlanBuilding(a.plan, plain, 0.0);
    auto partVerts = [](const BuildingMesh& bm, PartId id) {
        std::size_t n = 0;
        for (const RenderMesh& part : bm.parts) if (part.materialIndex == static_cast<int>(id)) n += part.vertices.size();
        return n;
    };
    CHECK(partVerts(ma, PartId::Concrete) > partVerts(mp, PartId::Concrete));   // the bridge's floor
    // the graph: a Link cell off storey 16, reached; everything else reached as before
    const SpaceGraph ga = spaceGraphOf(a.plan, a.params, 0.0, &ma.roof);
    int links = 0;
    for (const Cell& c : ga.cells) if (c.kind == CellKind::Link) { ++links; CHECK(c.k0 == 16); CHECK(c.freeArea > 4.5 * 12.0 * 0.9); }
    CHECK(links == 1);
    CHECK(unreachableCells(ga).empty());
    const SpaceGraph gb = spaceGraphOf(b.plan, b.params, 0.0);
    int linksB = 0;
    for (const Cell& c : gb.cells) if (c.kind == CellKind::Link) ++linksB;
    CHECK(linksB == 1);
    CHECK(unreachableCells(gb).empty());
    // the collider: the tier wall on a's bridge face is cut round the door (more quads than the plain wall)
    std::vector<Vec3> V0, V1;
    std::vector<uint32_t> I0, I1;
    appendBuildingTiers(V0, I0, a.plan, plain, 0.0, 0.0, {}, {}, {});
    appendBuildingTiers(V1, I1, a.plan, a.params, 0.0, 0.0, {}, {}, {});
    CHECK(I1.size() > I0.size());
    std::printf("    [twins] bridge at storey %d spanning %.1f m, %.1f m wide; a's roof items %d + %d, link cells %d / %d, collider %zu -> %zu indices\n",
                br.storey, br.span, br.width, bridgeItems, wallItems, links, linksB, I0.size(), I1.size());
}

// WIDE OPENINGS AND THE MALL AS CELLS (ADR-0180): a shared wall absent along a run on chosen storeys, and the
// American mall as a spine between two anchors with a garage behind, every one its own building, opened into.
TEST_CASE(a_wide_opening_leaves_the_shared_wall_absent_on_its_storeys) {
    const Poly2 box = {{-30, -15}, {30, -15}, {30, 15}, {-30, 15}};
    BuildingParams p = towerParams(6);
    p.faceDir = Vec3(0, 0, -1);
    Complex cx = bundledTubes(box, p, 2, 1, {6, 6});
    CHECK(cx.cells.size() == 2 && cx.adj.size() == 1);
    const Real run = (cx.adj[0].p1 - cx.adj[0].p0).length();
    CHECK(std::fabs(run - 30.0) < 1e-6);
    cx.openWall(0, run * 0.5 - 6.0, run * 0.5 + 6.0, 0, 2);
    CHECK(cx.adj[0].openings.size() >= 1);
    for (const CellFrame& c : cx.cells) {
        // both cells know the run on that wall on storeys 0..2, and the party door it covers is gone
        int open = 0;
        for (int k = 0; k < c.params.partyWalls; ++k)
            if (c.params.partyOpenK1[k] >= c.params.partyOpenK0[k]) { ++open; CHECK(c.params.partyOpenK0[k] == 0 && c.params.partyOpenK1[k] == 2); CHECK(std::fabs(c.params.partyOpen1[k] - c.params.partyOpen0[k] - 12.0) < 1e-6); }
        CHECK(open == 1);
        CHECK(c.params.partyDoors == 0);
        // the facade: on storeys 0..2 the party edge is open across 12 m, on 3 it is not
        Vec2 o0, o1;
        int openEdges = 0;
        for (std::size_t e = 0; e < c.plan.size(); ++e) if (partyOpenAt(c.plan, c.params, e, 1, o0, o1)) { ++openEdges; CHECK(std::fabs((o1 - o0).length() - 12.0) < 1e-6); }
        CHECK(openEdges == 1);
        int closedAbove = 0;
        for (std::size_t e = 0; e < c.plan.size(); ++e) closedAbove += partyOpenAt(c.plan, c.params, e, 3, o0, o1);
        CHECK(closedAbove == 0);
        // the cuts: one, floor 0 to the ceiling of storey 2
        const std::vector<WallCut> cuts = wallCuts(c.plan, c.params, 0.0);
        CHECK(cuts.size() == 1);
        if (!cuts.empty()) { CHECK(std::fabs(cuts[0].y0) < 1e-6); CHECK(std::fabs(cuts[0].y1 - (c.params.groundHeight + 2 * c.params.floorHeight)) < 1e-6); CHECK(!cuts[0].link); }
        // the collider's wall is in pieces; the graph has a Link cell on each open storey; everything is walked
        BuildingParams shut = c.params;
        for (int k = 0; k < 4; ++k) shut.partyOpenK1[k] = -1;
        std::vector<Vec3> V0, V1;
        std::vector<uint32_t> I0, I1;
        appendBuildingTiers(V0, I0, c.plan, shut, 0.0, 0.0, {}, {}, {});
        appendBuildingTiers(V1, I1, c.plan, c.params, 0.0, 0.0, {}, {}, {});
        CHECK(I1.size() > I0.size());
        const SpaceGraph g = spaceGraphOf(c.plan, c.params, 0.0);
        int links = 0;
        for (const Cell& cl : g.cells) if (cl.kind == CellKind::Link) { ++links; CHECK(cl.k0 >= 0 && cl.k0 <= 2); }
        CHECK(links == 3);
        CHECK(unreachableCells(g).empty());
        // the furniture keeps the run clear: a door-type opening the run's width on the open storey
        int clear = 0;
        for (const WallOpening& o : storeyOpenings(c.plan, c.params, 1, c.params.groundHeight, c.params.floorHeight, entranceEdgeFor(c.plan, c.params)))
            if (o.door && std::fabs((o.b - o.a).length() - 12.0) < 0.1) ++clear;
        CHECK(clear == 1);
    }
    std::printf("    [opening] a 12 m run open on storeys 0..2 of both tubes, closed on 3\n");
}

TEST_CASE(the_mall_as_cells_is_a_spine_two_anchors_and_a_garage_opened_into) {
    const Poly2 box = {{-90, -40}, {90, -40}, {90, 40}, {-90, 40}};
    const BuildingParams mall = architectMallWings(5).params;
    const BuildingParams anchor = architectBigBox(9).params;
    BuildingRecipe gr;
    CHECK(architectRecipeByName("parking_garage", 11, 0.3, 40.0, 1600.0, gr));
    const Complex cx = mallCells(box, Vec2(0, -1), mall, anchor, gr.params);
    CHECK(cx.cells.size() == 4);
    int spine = -1, anchors = 0, garage = -1;
    for (std::size_t i = 0; i < cx.cells.size(); ++i) {
        if (cx.cells[i].role == "spine") spine = static_cast<int>(i);
        if (cx.cells[i].role == "anchor") ++anchors;
        if (cx.cells[i].role == "garage") garage = static_cast<int>(i);
    }
    CHECK(spine >= 0 && anchors == 2 && garage >= 0);
    CHECK(cx.adj.size() == 3);   // two anchor walls, the garage's
    int anchorOpen = 0, garageOpen = 0;
    for (const Adjacency& ad : cx.adj)
        for (const Opening& o : ad.openings) {
            if (o.k1 == mall.floors && std::fabs(o.u1 - o.u0 - 12.0) < 1e-6) ++anchorOpen;
            if (o.k0 == 0 && o.k1 == 0 && std::fabs(o.u1 - o.u0 - 8.0) < 1e-6) ++garageOpen;
        }
    CHECK(anchorOpen == 2 && garageOpen == 1);
    const CellFrame& sp = cx.cells[static_cast<std::size_t>(spine)];
    CHECK(sp.params.bigBox == 6 && sp.params.mallArms == 0x3 && sp.params.mallOpenEnds == 0x3);
    CHECK(sp.params.partyWalls == 3);
    // the spine's levels: open-ended wings, no anchor rooms inside, the concourse to the end walls; its own
    // escalators in the court; every cell a building walked, the anchors two levels like the mall
    for (int k = 0; k <= 1; ++k) {
        const StoreyRooms sr = storeyRooms(sp.plan, sp.params, 0.0, storeyPlans(sp.plan, sp.params), k, coreFor(sp.plan, sp.params, entranceEdgeFor(sp.plan, sp.params)),
                                           stairRuns(sp.plan, sp.params, entranceEdgeFor(sp.plan, sp.params), storeyPlans(sp.plan, sp.params)), entranceEdgeFor(sp.plan, sp.params));
        int anchorRooms = 0, concourse = 0;
        for (const Room& r : sr.rp.rooms) { if (r.style == 10) ++anchorRooms; if (r.style == 21 || r.style == 22) ++concourse; }
        CHECK(anchorRooms == 0);
        CHECK(concourse == 2);
    }
    for (const CellFrame& c : cx.cells) {
        CHECK(c.connected);
        const SpaceGraph g = spaceGraphOf(c.plan, c.params, 0.0);
        CHECK(unreachableCells(g).empty());
        if (c.role == "anchor") { CHECK(c.params.floors == mall.floors); CHECK(c.params.groundHeight == mall.groundHeight); }
    }
    std::printf("    [mall] spine %.0f x %.0f, %d anchors, a garage; %zu shared walls, %d anchor openings, %d garage opening\n",
                2 * orientedBoundingBox(sp.plan).half[orientedBoundingBox(sp.plan).longAxis()], 2 * orientedBoundingBox(sp.plan).half[1 - orientedBoundingBox(sp.plan).longAxis()],
                anchors, cx.adj.size(), anchorOpen, garageOpen);
}
