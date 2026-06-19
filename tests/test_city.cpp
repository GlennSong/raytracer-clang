#include "test_framework.h"

#include "../src/engine/procgen/city/polygon.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/parcel.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/city.h"
#include <algorithm>
#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

namespace {
Poly2 square(Real s) {
    return {{0, 0}, {s, 0}, {s, s}, {0, s}};   // CCW
}
bool hasPart(const BuildingMesh& bm, PartId id) {
    for (const RenderMesh& p : bm.parts)
        if (p.materialIndex == static_cast<int>(id)) return true;
    return false;
}
}  // namespace

// --- polygon ----------------------------------------------------------------

TEST_CASE(polygon_area_centroid_and_winding) {
    Poly2 sq = square(10);
    CHECK_APPROX(area(sq), 100.0, 1e-9);
    CHECK(isCCW(sq));
    Vec2 c = centroid(sq);
    CHECK_APPROX(c.x, 5.0, 1e-9);
    CHECK_APPROX(c.y, 5.0, 1e-9);
    // A CW ring has negative signed area; ensureCCW flips it.
    Poly2 cw = {{0, 0}, {0, 10}, {10, 10}, {10, 0}};
    CHECK(signedArea(cw) < 0);
    ensureCCW(cw);
    CHECK(isCCW(cw));
}

TEST_CASE(polygon_point_in_polygon) {
    Poly2 sq = square(10);
    CHECK(pointInPolygon(sq, {5, 5}));
    CHECK(!pointInPolygon(sq, {15, 5}));
    CHECK(!pointInPolygon(sq, {-1, 5}));
}

TEST_CASE(polygon_convex_hull) {
    Poly2 pts = {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {5, 5}, {3, 7}};  // +2 interior
    Poly2 hull = convexHull(pts);
    CHECK(hull.size() == 4);                 // interior points dropped
    CHECK_APPROX(area(hull), 100.0, 1e-9);
}

TEST_CASE(polygon_obb_of_axis_aligned_rect) {
    Poly2 rect = {{0, 0}, {20, 0}, {20, 6}, {0, 6}};
    OBB2 obb = orientedBoundingBox(rect);
    CHECK_APPROX(obb.center.x, 10.0, 1e-6);
    CHECK_APPROX(obb.center.y, 3.0, 1e-6);
    // Long axis half-extent ~10, short ~3.
    Real lo = obb.half[obb.longAxis()], sh = obb.half[1 - obb.longAxis()];
    CHECK_APPROX(lo, 10.0, 1e-6);
    CHECK_APPROX(sh, 3.0, 1e-6);
}

TEST_CASE(polygon_inset_shrinks_area) {
    Poly2 sq = square(20);
    Poly2 in = inset(sq, 2.0);
    CHECK(in.size() == 4);
    // Inset of a 20x20 square by 2 -> 16x16.
    CHECK_APPROX(area(in), 256.0, 1e-6);
    // Over-inset past the medial axis collapses to empty.
    CHECK(inset(sq, 11.0).empty());
}

TEST_CASE(polygon_split_by_line) {
    Poly2 sq = square(10);
    Poly2 left, right;
    splitByLine(sq, {5, 5}, {0, 1}, left, right);   // vertical line x=5
    CHECK(left.size() >= 3);
    CHECK(right.size() >= 3);
    CHECK_APPROX(area(left) + area(right), 100.0, 1e-6);
    CHECK_APPROX(area(left), 50.0, 1e-6);
}

// --- shape grammar (Phase 0) ------------------------------------------------

TEST_CASE(grammar_grows_a_multipart_building) {
    BuildingParams p; p.floors = 5; p.seed = 3;
    Scope s = scopeFromFootprint(square(18), 0.0, 20.0);
    BuildingMesh bm = growBuilding(s, p);
    CHECK(bm.parts.size() >= 3);                 // wall + glass + roof at least
    CHECK(hasPart(bm, PartId::Wall));
    CHECK(hasPart(bm, PartId::Glass));
    // 5 floors * 3.2 + 4.5 ground + 1.1 parapet = 21.6 m.
    CHECK_APPROX(bm.height, 5 * 3.2 + 4.5 + 1.1, 1e-6);
    CHECK(!bm.proxy.vertices.empty());           // coarse LOD proxy emitted
    CHECK(!bm.attaches.empty());
}

TEST_CASE(grammar_walkable_ground_punches_a_door) {
    BuildingParams p; p.floors = 3; p.walkableGround = true; p.seed = 1;
    Scope s = scopeFromFootprint(square(16), 0.0, 12.0);
    BuildingMesh bm = growBuilding(s, p);
    CHECK(hasPart(bm, PartId::Door));            // a real entrance opening
    bool entrance = false;
    for (const AttachPoint& a : bm.attaches) if (a.tag == "entrance") entrance = true;
    CHECK(entrance);
}

TEST_CASE(grammar_taller_with_more_floors_and_deterministic) {
    Scope s = scopeFromFootprint(square(16), 0.0, 12.0);
    BuildingParams a; a.floors = 4; a.seed = 7;
    BuildingParams b; b.floors = 12; b.seed = 7;
    CHECK(growBuilding(s, b).height > growBuilding(s, a).height);
    // Same seed + params -> identical geometry.
    BuildingMesh m1 = growBuilding(s, a);
    BuildingMesh m2 = growBuilding(s, a);
    CHECK(m1.parts.size() == m2.parts.size());
    CHECK(m1.merged().vertices.size() == m2.merged().vertices.size());
}

// --- parcels (Phase 1) ------------------------------------------------------

TEST_CASE(parcel_subdivides_into_lots_conserving_area) {
    Poly2 block = square(100);                   // 10,000 m^2
    ParcelParams pp; pp.targetArea = 420; pp.seed = 5;
    std::vector<Lot> lots = subdivideBlock(block, pp);
    CHECK(lots.size() > 5);
    Real total = 0;
    for (const Lot& l : lots) {
        total += l.area;
        CHECK(l.area > 0);
        CHECK_APPROX(l.frontage.length(), 1.0, 1e-6);
    }
    CHECK_APPROX(total, 10000.0, 1.0);           // partition conserves area
}

TEST_CASE(parcel_is_deterministic) {
    Poly2 block = square(80);
    ParcelParams pp; pp.seed = 9;
    CHECK(subdivideBlock(block, pp).size() == subdivideBlock(block, pp).size());
}

// --- road network (Phase 2) -------------------------------------------------

TEST_CASE(road_grid_blocks_equal_cells) {
    GridRoadParams gp; gp.extent = 200; gp.cellSize = 100; gp.jitter = 0; gp.seed = 1;
    RoadGraph g = gridRoads(gp);
    CHECK(g.nodes.size() == 25);                 // 5x5 node grid
    RoadGraph pg = planarize(g);
    std::vector<Poly2> blocks = extractBlocks(pg);
    CHECK(blocks.size() == 16);                  // 4x4 cells
    Real total = 0;
    for (const Poly2& b : blocks) { total += area(b); CHECK(isCCW(b)); }
    CHECK_APPROX(total, 400.0 * 400.0, 1.0);     // faces tile the region exactly
}

TEST_CASE(road_planarize_splits_a_crossing) {
    // Two crossing segments forming an X: planarize must insert the centre node
    // and split both edges into 4.
    RoadGraph g;
    int a = g.addNode({-10, 0}), b = g.addNode({10, 0});
    int c = g.addNode({0, -10}), d = g.addNode({0, 10});
    g.addEdge(a, b); g.addEdge(c, d);
    RoadGraph pg = planarize(g);
    CHECK(pg.nodes.size() == 5);                 // 4 ends + 1 crossing
    CHECK(pg.edges.size() == 4);                 // each segment split in two
}

// --- city (Phase 3) ---------------------------------------------------------

TEST_CASE(city_generates_deterministically) {
    CityParams cp; cp.extent = 300; cp.cellSize = 100; cp.seed = 42;
    CityModel a = generateCity(cp);
    CityModel b = generateCity(cp);
    CHECK(a.buildings.size() == b.buildings.size());
    CHECK(a.blockCount == b.blockCount);
    CHECK(a.buildings.size() > 0);
    CHECK(!a.parts.empty());
}

TEST_CASE(city_highrise_is_taller_than_residential) {
    CityParams cp; cp.extent = 400; cp.cellSize = 95; cp.seed = 7;
    CityModel m = generateCity(cp);
    Real maxHigh = 0, maxResidential = 0;
    bool industrial = false;
    for (const CityBuilding& b : m.buildings) {
        if (b.district == District::HighRise) maxHigh = std::max(maxHigh, b.height);
        if (b.district == District::Residential) maxResidential = std::max(maxResidential, b.height);
        if (b.district == District::Industrial) industrial = true;
    }
    CHECK(maxHigh > maxResidential);
    CHECK(maxHigh > 40.0);                         // high-rise towers exist
    CHECK(industrial);                             // the industrial zone is populated
}

TEST_CASE(city_drapes_on_terrain_foundations_track_ground) {
    // City Arena (ADR-0038 §6): with a ground sampler, foundations sit on the
    // terrain (a ramp -> a spread of base elevations), there is no flat ground
    // plane, and street/park trees are scattered.
    CityParams cp; cp.extent = 280; cp.cellSize = 95; cp.seed = 4;
    cp.groundAt = [](const Vec2& p) { return 0.5 * p.x; };   // linear ramp
    CityModel m = generateCity(cp);
    CHECK(m.ground.vertices.empty());          // terrain is the ground
    CHECK(m.treeCount > 0);
    CHECK(!m.props.vertices.empty());

    Real lo = 1e30, hi = -1e30;
    for (const CityBuilding& b : m.buildings) {
        lo = std::min(lo, b.baseY); hi = std::max(hi, b.baseY);
        // The building sits on its block's flat grade (max ground over the whole
        // block), so it tracks the ramp within a block half-width (~50 m).
        CHECK(std::fabs(b.baseY - 0.5 * b.site.x) < 60.0);
    }
    CHECK(hi - lo > 50.0);                      // buildings span the ramp
}

TEST_CASE(city_flat_keeps_ground_plane_and_trees) {
    CityParams cp; cp.extent = 200; cp.cellSize = 95; cp.seed = 2;
    CityModel m = generateCity(cp);            // no sampler -> flat
    CHECK(!m.ground.vertices.empty());         // flat city gets a ground plane
    CHECK(m.treeCount > 0);
    // Flat ground: every building sits on the block grade, the curb +0.15 lift.
    for (const CityBuilding& b : m.buildings) CHECK_APPROX(b.baseY, 0.15, 1e-6);
}

TEST_CASE(city_buildings_have_valid_box_colliders) {
    CityParams cp; cp.extent = 200; cp.cellSize = 95; cp.seed = 5;
    cp.groundAt = [](const Vec2& p) { return 0.3 * p.x; };   // sloped, to vary baseY
    CityModel m = generateCity(cp);
    CHECK(!m.buildings.empty());
    for (const CityBuilding& b : m.buildings) {
        CHECK(b.boxHalf.x > 0.1 && b.boxHalf.y > 0.1 && b.boxHalf.z > 0.1);
        // Box centre sits half the height above the foundation.
        CHECK_APPROX(b.boxCenter.y, b.baseY + b.height * 0.5, 1e-6);
        // Box centre (the OBB centre) is near the footprint site.
        CHECK(distance(Vec2(b.boxCenter.x, b.boxCenter.z), b.site) < 6.0);
    }
}

TEST_CASE(city_hlod_proxy_is_far_cheaper_than_detail) {
    CityParams cp; cp.extent = 300; cp.cellSize = 100; cp.seed = 11;
    CityModel m = generateCity(cp);
    std::size_t detailTris = 0;
    for (const RenderMesh& p : m.parts) detailTris += p.indices.size() / 3;
    std::size_t proxyTris = m.hlodProxy.indices.size() / 3;
    CHECK(proxyTris > 0);
    CHECK(proxyTris == m.buildings.size() * 12);   // one 12-tri box per building
    // The whole-city HLOD is an order of magnitude lighter than the detail.
    CHECK(proxyTris * 10 < detailTris);
}

TEST_CASE(city_parks_leave_blocks_empty) {
    CityParams cp; cp.extent = 400; cp.cellSize = 95; cp.parkFraction = 0.3; cp.seed = 3;
    CityModel m = generateCity(cp);
    // With 30% parks, fewer buildings than a no-park run of the same seed.
    CityParams cp2 = cp; cp2.parkFraction = 0.0;
    CityModel m2 = generateCity(cp2);
    CHECK(m.buildings.size() < m2.buildings.size());
}
