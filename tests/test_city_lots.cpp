#include "test_framework.h"

#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/shape_grammar.h"   // PartId (surfaced walls)

using namespace engine;

// Living City, ADR-0066: buildings grown on the road network's blocks. The pass
// is pure geometry (blocks → lots → typed buildings), so its guarantees — real
// buildings come out, none are slivers, tagged for schedules, deterministic — are
// pinned headless; the visible spawn is device-gated.

namespace {
// A few big square blocks the parcel step can actually subdivide.
std::vector<Poly2> squareBlocks() {
    std::vector<Poly2> blocks;
    for (int i = 0; i < 4; ++i) {
        Real cx = (i % 2) * 120.0 - 60.0, cz = (i / 2) * 120.0 - 60.0;
        Real h = 45.0;   // 90 m block
        blocks.push_back({{cx - h, cz - h}, {cx + h, cz - h},
                          {cx + h, cz + h}, {cx - h, cz + h}});
    }
    return blocks;
}
bool isKnownType(const std::string& t) {
    return t == "home" || t == "shop" || t == "office" || t == "civic" || t == "park";
}
}  // namespace

TEST_CASE(lot_buildings_are_grown_and_not_slivers) {
    LotParams p;
    p.center = {0, 0};
    p.seed = 3;
    std::vector<RenderMesh> parts;
    std::vector<LotBuilding> b = growLotBuildings(squareBlocks(), p, nullptr, &parts);
    CHECK(!b.empty());   // real blocks yield buildings
    for (const LotBuilding& lb : b) {
        const Real shortSide = std::min(lb.width, lb.depth);
        const Real longSide = std::max(lb.width, lb.depth);
        CHECK(shortSide >= p.minShort - 1e-6);          // no knife blades
        CHECK(longSide <= shortSide * p.maxAspect + 1e-6);
        CHECK(lb.height > 0.0);                         // has mass (park = low pad)
        CHECK(isKnownType(lb.type));                    // a valid schedule tag
    }
    // The buildings' geometry lands in the per-PartId meshes with the part id set
    // (the loader binds the shape-grammar PBR recipes onto exactly these).
    CHECK(!parts.empty());
    int filled = 0;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (parts[i].vertices.empty()) continue;
        ++filled;
        CHECK(parts[i].materialIndex == static_cast<int>(i));
    }
    CHECK(filled >= 2);   // at least walls + one more class (glass/roof/trim)
    // Facades must land in the SURFACED wall parts (Brick/Concrete/Stucco/Metal
    // carry the procedural PBR texture recipes) — leaving every wall in the flat
    // PartId::Wall is the "buildings are colour-only" device bug.
    std::size_t surfacedVerts = 0;
    for (PartId id : {PartId::Brick, PartId::Concrete, PartId::Stucco, PartId::Metal})
        surfacedVerts += parts[static_cast<std::size_t>(id)].vertices.size();
    CHECK(surfacedVerts > 0);
}

TEST_CASE(lot_buildings_are_deterministic) {
    LotParams p;
    p.seed = 7;
    std::vector<LotBuilding> a = growLotBuildings(squareBlocks(), p);
    std::vector<LotBuilding> c = growLotBuildings(squareBlocks(), p);
    CHECK(a.size() == c.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].type == c[i].type);
        CHECK_APPROX(a[i].site.x, c[i].site.x, 1e-9);
        CHECK_APPROX(a[i].height, c[i].height, 1e-9);
    }
    LotParams q = p; q.seed = 99;
    std::vector<LotBuilding> d = growLotBuildings(squareBlocks(), q);
    CHECK(!d.empty());
}

TEST_CASE(lot_buildings_zone_radially) {
    // Downtown (near the centre) should skew commercial/civic, not all houses.
    LotParams p;
    p.center = {-60, -60};   // put downtown on the first block's centre
    p.seed = 5;
    std::vector<LotBuilding> b = growLotBuildings(squareBlocks(), p);
    int nearNonHome = 0;
    for (const LotBuilding& lb : b) {
        if ((lb.site - p.center).length() > p.innerRadius) continue;
        if (lb.type != "home") nearNonHome++;
    }
    CHECK(nearNonHome > 0);
}

TEST_CASE(edge_blocks_build_the_town_rim) {
    // A single long straight boundary road, no enclosed faces anywhere: edge
    // blocks must appear on BOTH open sides, sized within [minLen, maxLen] and
    // set back by the margin, and the usual lot pass grows buildings on them.
    RoadGraph roads;
    roads.nodes = { {Vec2(0, 0)}, {Vec2(150, 0)} };
    roads.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    EdgeBlockParams ep;   // margin 9, depth 34, len 26..58
    std::vector<Poly2> rim = edgeBlocks(roads, {}, ep);
    CHECK(!rim.empty());
    CHECK(rim.size() % 2 == 0u);   // both sides of a road with open ground
    for (const Poly2& r : rim) {
        CHECK(r.size() == 4u);
        Vec2 lo, hi; bounds(r, lo, hi);
        const Real len = hi.x - lo.x, depth = hi.y - lo.y;
        CHECK(len >= ep.minLen * 0.6 - 1e-6);
        CHECK(len <= ep.maxLen + 1e-6);
        CHECK_APPROX(depth, ep.depth, 1e-6);
        // Set back from the road (y=0) by the margin: no corner nearer than it.
        CHECK(std::min(std::abs(lo.y), std::abs(hi.y)) >= ep.margin - 1e-6);
    }
    // The rim rectangles grow buildings through the ordinary lot pass.
    LotParams lp; lp.seed = 9; lp.roadMargin = 2.0;   // rim blocks are pre-set-back
    CHECK(!growLotBuildings(rim, lp).empty());
}

TEST_CASE(edge_blocks_skip_covered_ground) {
    // The same road, but one side is an enclosed city block: only the open side
    // gets rim rectangles.
    RoadGraph roads;
    roads.nodes = { {Vec2(0, 0)}, {Vec2(150, 0)} };
    roads.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    Poly2 block{{-10, 5}, {160, 5}, {160, 80}, {-10, 80}};   // covers +y side
    EdgeBlockParams ep;
    std::vector<Poly2> rim = edgeBlocks(roads, {block}, ep);
    CHECK(!rim.empty());
    for (const Poly2& r : rim) {
        Vec2 lo, hi; bounds(r, lo, hi);
        CHECK(hi.y < 0);   // every rim block sits on the OPEN (-y) side
    }
}
